/*****************************************************************************
 * read.c: block reads
 *****************************************************************************
 * Copyright (C) 2026 Authors
 *
 * Authors: Pierre Bogdanovscky
 * Co-authored-by: claude-code:claude-opus-5-5
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston MA 02110-1301, USA.
 *****************************************************************************
 * laser_read_blocks() and laser_read_cd_blocks(): a read of any length cut
 * into transfers the bridge accepts, through laser_scsi_cdb(), and what the
 * reads learn of the device on the way - the largest transfer it takes, and
 * the blocks known not to read (laser_unreadable_blocks(),
 * laser_refused_for_good()), kept until the disc changes.
 *****************************************************************************/

#include <string.h>

#include "laser.h"
#include "laser_internal.h"
#include "scsi_internal.h"

/* ============================================================================
 * Public: LBA-aware chunked block reads
 * ============================================================================ */

/* A chunk the drive answered with a medium error is narrowed down to the
 * blocks that do not read - see read_chunked() - down to this many blocks,
 * and for this long at most from the first failure: a bridge stalling
 * seconds on each medium error would otherwise turn a scratch into half a
 * minute of halving. */
#define LASER_MEDIUM_NARROW_MIN_BLOCKS 4
#define LASER_MEDIUM_NARROW_MAX_MS     5000

/* Build the CDB for one chunk - the only thing the two public helpers below
 * do differently. */
typedef void (*build_read_cdb_fn)(uint8_t *cdb, uint32_t lba, int blocks,
                                  const void *ctx);

/* Record that this device completed a @p bytes read in one command; see
 * laser_entry_t::largest_ok_bytes. */
static void raise_largest_ok(laser_entry_t *entry, int bytes)
{
    if (bytes <= entry->largest_ok_bytes) {
        return;
    }

    pthread_mutex_lock(&entry->io_lock);
    if (bytes > entry->largest_ok_bytes) {
        entry->largest_ok_bytes = bytes;
    }
    pthread_mutex_unlock(&entry->io_lock);
}

/* Lower this device's transfer cap to @p bytes, if it is above that. Called
 * once smaller transfers have read the whole range a @p failed_bytes one
 * failed on. */
static void lower_transfer_cap(laser_entry_t *entry, int bytes,
                               int failed_bytes)
{
    pthread_mutex_lock(&entry->io_lock);

    const int lowered = bytes < entry->max_transfer_bytes;
    if (lowered) {
        entry->max_transfer_bytes = bytes;
    }

    pthread_mutex_unlock(&entry->io_lock);

    if (lowered) {
        LOG_QUIRK(entry, "a %d-byte transfer failed where %d-byte ones read "
                         "the same blocks, limiting this device to %d bytes "
                         "per command from now on",
                  failed_bytes, bytes, bytes);
    }
}

/* Before a chunk of @p *chunk blocks at @p lba, against the blocks the disc
 * is known not to read: laser_entry_t::unreadable, and with @p refusals
 * laser_entry_t::refused.
 *
 * Returns 0 if the chunk may go to the drive, shortened to stop before the
 * first known block after @p lba - or to that one block, for a refusal not
 * yet answered LASER_REFUSAL_MAX_HITS times, so that it is asked again and
 * counted. Otherwise the status to answer without the drive: LASER_ERR_IO in
 * an unreadable run, the refusal's status on a block refused that often. */
static int known_bad_block(laser_entry_t *entry, uint32_t lba, int *chunk,
                           int refusals)
{
    int status = 0;

    pthread_mutex_lock(&entry->io_lock);
    for (unsigned i = 0; i < LASER_UNREADABLE_SLOTS && status == 0; i++) {
        const uint32_t bad = entry->unreadable[i].lba;
        const uint32_t count = entry->unreadable[i].count;
        if (count == 0) {
            continue;
        }
        if (lba >= bad && lba - bad < count) {
            status = (int)LASER_ERR_IO;
        } else if (bad > lba && bad - lba < (uint32_t)*chunk) {
            *chunk = (int)(bad - lba);
        }
    }
    for (unsigned i = 0; refusals && i < LASER_REFUSAL_SLOTS && status == 0;
         i++) {
        const uint32_t bad = entry->refused[i].lba;
        if (entry->refused[i].hits == 0) {
            continue;
        }
        if (bad == lba) {
            if (entry->refused[i].hits >= LASER_REFUSAL_MAX_HITS) {
                status = entry->refused[i].status;
            } else {
                *chunk = 1;
            }
        } else if (bad > lba && bad - lba < (uint32_t)*chunk) {
            *chunk = (int)(bad - lba);
        }
    }
    pthread_mutex_unlock(&entry->io_lock);

    return status;
}

static void remember_unreadable(laser_entry_t *entry, int token, uint32_t lba,
                                int count)
{
    pthread_mutex_lock(&entry->io_lock);
    entry->unreadable[entry->unreadable_next].lba = lba;
    entry->unreadable[entry->unreadable_next].count = (uint32_t)count;
    entry->unreadable_next = (entry->unreadable_next + 1) %
                             LASER_UNREADABLE_SLOTS;
    pthread_mutex_unlock(&entry->io_lock);

    LOGW("token=%d: LBA %u x %d unreadable, not asked for again until the "
         "disc changes", token, lba, count);
}

/* A READ(10) of the single block @p lba was refused with @p status. Logged
 * once per block. */
static void remember_refusal(laser_entry_t *entry, int token, uint32_t lba,
                             int status)
{
    pthread_mutex_lock(&entry->io_lock);
    for (unsigned i = 0; i < LASER_REFUSAL_SLOTS; i++) {
        if (entry->refused[i].hits > 0 && entry->refused[i].lba == lba) {
            /* Saturating: a wrap would unlatch. */
            if (entry->refused[i].hits < LASER_REFUSAL_MAX_HITS) {
                entry->refused[i].hits++;
            }
            pthread_mutex_unlock(&entry->io_lock);
            return;
        }
    }

    /* The least-refused block makes way. */
    unsigned slot = 0;
    for (unsigned i = 1; i < LASER_REFUSAL_SLOTS; i++) {
        if (entry->refused[slot].hits == 0) {
            break;      /* an empty slot is as good as it gets */
        }
        if (entry->refused[i].hits < entry->refused[slot].hits) {
            slot = i;
        }
    }
    entry->refused[slot].lba = lba;
    entry->refused[slot].status = status;
    entry->refused[slot].hits = 1;
    pthread_mutex_unlock(&entry->io_lock);

    switch (status) {
    case LASER_ERR_REGION:
        LOGE("token=%d: this disc's region does not match the drive's. This "
             "is NOT an authentication problem and no key exchange will fix "
             "it.", token);
        break;
    case LASER_ERR_SCRAMBLED:
        LOGW("token=%d: the drive refuses to read LBA %u - CSS-scrambled "
             "content on a drive that has not been authenticated. Reads "
             "elsewhere on the disc are unaffected.", token, lba);
        break;
    default:
        LOGW("token=%d: the drive refuses to read LBA %u and will not explain "
             "why (status %d). Reads elsewhere on the disc are unaffected.",
             token, lba, status);
        break;
    }
}

static void forget_refusals(laser_entry_t *entry, int token, const char *why)
{
    int count = 0;

    pthread_mutex_lock(&entry->io_lock);
    for (unsigned i = 0; i < LASER_REFUSAL_SLOTS; i++) {
        count += entry->refused[i].hits > 0;
    }
    memset(entry->refused, 0, sizeof(entry->refused));
    pthread_mutex_unlock(&entry->io_lock);

    if (count > 0) {
        LOGI("token=%d: %s - forgetting %d refused block(s)", token, why,
             count);
    }
}

/* See scsi_internal.h. */
void laser_forget_disc(laser_entry_t *entry, int token)
{
    pthread_mutex_lock(&entry->io_lock);
    memset(entry->unreadable, 0, sizeof(entry->unreadable));
    entry->unreadable_next = 0;
    pthread_mutex_unlock(&entry->io_lock);

    forget_refusals(entry, token, "the disc changed");
}

int laser_unreadable_blocks(int token, uint32_t lba, int num_blocks)
{
    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL || num_blocks <= 0) {
        return 0;
    }

    int count = 0;
    pthread_mutex_lock(&entry->io_lock);
    for (unsigned i = 0; i < LASER_UNREADABLE_SLOTS; i++) {
        const uint32_t bad = entry->unreadable[i].lba;
        const uint32_t run = entry->unreadable[i].count;
        if (run != 0 && lba >= bad && lba - bad < run) {
            const uint32_t left = run - (lba - bad);
            count = left < (uint32_t)num_blocks ? (int)left : num_blocks;
            break;
        }
    }
    pthread_mutex_unlock(&entry->io_lock);

    return count;
}

int laser_refused_for_good(int token, uint32_t lba)
{
    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL) {
        return 0;
    }

    int latched = 0;
    pthread_mutex_lock(&entry->io_lock);
    for (unsigned i = 0; i < LASER_REFUSAL_SLOTS; i++) {
        if (entry->refused[i].hits >= LASER_REFUSAL_MAX_HITS &&
            entry->refused[i].lba == lba) {
            latched = 1;
            break;
        }
    }
    pthread_mutex_unlock(&entry->io_lock);

    return latched;
}

/* The body shared by laser_read_blocks() and laser_read_cd_blocks(): reads
 * @p num_blocks blocks from @p lba into @p buffer, one command per chunk that
 * fits the device's transfer cap.
 *
 * Returns:
 *
 *   > 0  blocks read, contiguous from @p lba. May be fewer than asked: a
 *        short read is the ordinary answer for a scratched sector, and
 *        callers advance by what came back and ask for the rest.
 *   = 0  only when @p num_blocks was <= 0. A read that brought back nothing
 *        is LASER_ERR_IO, since 0 would make those callers ask again
 *        forever.
 *   < 0  a laser_status_t. Whatever is already in @p buffer must be
 *        ignored, including the chunks that succeeded.
 *
 * MEDIA_GONE, NO_DEVICE and CANCELLED are returned even after successful
 * chunks: there is nothing left to read around, or nobody left asking, and a
 * caller given a short read would keep walking a drive that cannot answer -
 * dvdread and dvdnav treat a short read as an ordinary disc imperfection.
 * Any other failure becomes a short read once something has been read.
 *
 * TRANSFER-SIZE NEGOTIATION. Some USB-ATAPI bridges fail a large data phase
 * outright, where the same blocks read in smaller commands go through. So a
 * plain I/O failure on more than LASER_MIN_BYTES_PER_TRANSFER is not final:
 * the same blocks are read again at half the size, halving again on each
 * failure. Only once the whole failed range has been read that way is the
 * smaller size kept for the device: the large command failed on blocks that
 * are readable, so the limit was the bridge's. A scratch inside the range
 * fails the smaller reads too, and leaves the cap alone.
 *
 * This terminates: every retry is strictly smaller than the transfer that
 * failed, and none goes below LASER_MIN_BYTES_PER_TRANSFER.
 *
 * BLOCKS KNOWN NOT TO READ are not asked for again. A chunk failing with
 * LASER_ERR_IO, its retries spent, is recorded unreadable for the device
 * (laser_entry_t::unreadable): a read starting in it fails at once, and one
 * reaching it stops short of it, as a short read. With @p refusals - READ(10)
 * - a chunk refused with a positional status is read again at half the size
 * down to the block refused, so that the blocks before it are returned, and
 * that block is recorded (laser_entry_t::refused); a read reaching it stops
 * short of it, and after LASER_REFUSAL_MAX_HITS refusals one starting on it
 * gets the status without the drive. Both are forgotten when the disc
 * changes, the refusals also when the drive reports no CSS session.
 *
 * A MEDIUM ERROR IS NARROWED DOWN. A chunk the drive answered with a medium
 * error - it reached the sector and could not read it - is read again at
 * half the size, each smaller read tried once, so that only the blocks that
 * do not read are recorded and lost, not a whole chunk: a fraction of a
 * second of silence on a scratched CD rather than a third of a second, a
 * glitch in a film rather than a whole read-ahead window. Down to
 * LASER_MEDIUM_NARROW_MIN_BLOCKS blocks, for LASER_MEDIUM_NARROW_MAX_MS at
 * most, and never after a drive that did not answer at all: a protection's
 * unreadable sectors are those, each costing the drive a timeout.
 *
 * @param token      registry token; the caller must hold a claim on it
 * @param lba        first block to read
 * @param num_blocks how many to read; <= 0 reads nothing and succeeds
 * @param block_size bytes per block, which also sets how many fit in one
 *                   transaction
 * @param cdb_len    10 for READ(10), 12 for READ CD
 * @param build_cdb  fills a zeroed 16-byte CDB for one chunk
 * @param ctx        handed to @p build_cdb untouched
 * @param buffer     at least @p num_blocks * @p block_size bytes
 * @param what       what to call these in a log line ("blocks", "sectors")
 * @param refusals   whether refusals are narrowed down and recorded: for
 *                   READ(10), the 2048-byte blocks of DVDs and BDs
 */
static int read_chunked(int token, uint32_t lba, int num_blocks,
                        int block_size, int cdb_len,
                        build_read_cdb_fn build_cdb, const void *ctx,
                        uint8_t *buffer, const char *what, int refusals)
{
    if (num_blocks <= 0) {
        return 0;
    }

    /* For the transfer cap. A miss costs only the negotiation:
     * laser_scsi_cdb() looks the token up again and reports it. */
    laser_entry_t *entry = laser_lookup(token);
    int blocks_done = 0;

    /* A negotiation in progress: trial_cap is the size being tried, 0 when
     * there is none; trial_end is the block index where the failed range
     * ends, and trial_failed the size that failed on it. */
    int trial_cap = 0;
    int trial_end = 0;
    int trial_failed = 0;

    /* The size a refusal was narrowed down to, 0 when none: the blocks
     * before the refused one are read in such chunks, for this read only. */
    int refusal_cap = 0;

    /* Likewise for a medium error, and when it was first answered, 0 when
     * none has been. */
    int medium_cap = 0;
    int64_t medium_since = 0;

    while (blocks_done < num_blocks) {
        /* Re-read every time round: another thread on the same drive may
         * have lowered it. */
        int cap = entry != NULL ? entry->max_transfer_bytes
                                : LASER_MAX_BYTES_PER_TRANSFER;
        if (trial_cap > 0 && trial_cap < cap) {
            cap = trial_cap;
        }
        if (refusal_cap > 0 && refusal_cap < cap) {
            cap = refusal_cap;
        }
        if (medium_cap > 0 && medium_cap < cap) {
            cap = medium_cap;
        }

        int max_blocks_per_chunk = cap / block_size;
        if (max_blocks_per_chunk < 1) {
            /* One block is the smallest read that makes progress. */
            max_blocks_per_chunk = 1;
        }

        int chunk = num_blocks - blocks_done;
        if (chunk > max_blocks_per_chunk) {
            chunk = max_blocks_per_chunk;
        }

        /* Blocks known not to read: answered without the drive. */
        if (entry != NULL) {
            const int known = known_bad_block(entry,
                                              lba + (uint32_t)blocks_done,
                                              &chunk, refusals);
            if (known != 0) {
                return blocks_done > 0 ? blocks_done : known;
            }
        }

        uint8_t cdb[16];
        memset(cdb, 0, sizeof(cdb));
        build_cdb(cdb, lba + (uint32_t)blocks_done, chunk, ctx);

        int actual_len = 0;
        int medium_error = 0;
        laser_status_t st = laser_scsi_cdb_read(
                token, cdb, cdb_len,
                buffer + (size_t)blocks_done * block_size,
                chunk * block_size, /* data_in = */ 1, &actual_len,
                medium_cap > 0 ? 1 : LASER_READ_MAX_MEDIUM_ERRORS,
                &medium_error);

        if (st != LASER_OK) {
            if (st == LASER_ERR_MEDIA_GONE || st == LASER_ERR_NO_DEVICE ||
                st == LASER_ERR_CANCELLED) {
                if (st == LASER_ERR_MEDIA_GONE && entry != NULL) {
                    laser_forget_disc(entry, token);
                }
                return (int)st;
            }

            /* A refusal fails the whole command: narrowed down to the
             * refused block, so that the blocks before it are returned. */
            if (refusals && laser_status_is_positional(st) &&
                entry != NULL) {
                if (chunk > 1) {
                    refusal_cap = (chunk / 2) * block_size;
                    continue;
                }
                remember_refusal(entry, token, lba + (uint32_t)blocks_done,
                                 (int)st);
                return blocks_done > 0 ? blocks_done : (int)st;
            }

            /* No CSS session in force: no block is to blame, and those
             * refused may read once libdvdcss authenticates again. */
            if (st == LASER_ERR_NO_KEY && entry != NULL) {
                forget_refusals(entry, token,
                                "the drive reports no CSS session in force");
            }

            /* The drive reached the blocks and could not read them: the
             * same blocks at half the size, so that those before the bad
             * ones are returned - see "a medium error is narrowed down"
             * above. The bridge is not to blame: it carried the answer. */
            if (st == LASER_ERR_IO && medium_error && entry != NULL) {
                const int64_t now = laser_monotonic_ms_now();
                if (medium_since == 0) {
                    medium_since = now;
                }
                if (chunk > LASER_MEDIUM_NARROW_MIN_BLOCKS &&
                    now - medium_since < LASER_MEDIUM_NARROW_MAX_MS) {
                    medium_cap = (chunk / 2) * block_size;
                    continue;
                }
                remember_unreadable(entry, token,
                                    lba + (uint32_t)blocks_done, chunk);
                return blocks_done > 0 ? blocks_done : (int)st;
            }

            /* Maybe the bridge rather than the disc: try the same blocks at
             * half the size. The first failure of a negotiation sets the
             * range it must cover. Not when the device has already read
             * this much in one command: then it is the disc. */
            const int tried = chunk * block_size;
            if (st == LASER_ERR_IO && chunk > 1 && entry != NULL &&
                tried > LASER_MIN_BYTES_PER_TRANSFER &&
                tried > entry->largest_ok_bytes) {
                if (trial_cap == 0) {
                    trial_end = blocks_done + chunk;
                    trial_failed = tried;
                }
                trial_cap = tried / 2;
                if (trial_cap < LASER_MIN_BYTES_PER_TRANSFER) {
                    trial_cap = LASER_MIN_BYTES_PER_TRANSFER;
                }
                continue;
            }

            if (st == LASER_ERR_IO && entry != NULL) {
                remember_unreadable(entry, token, lba + (uint32_t)blocks_done,
                                    chunk);
            }
            return blocks_done > 0 ? blocks_done : (int)st;
        }

        if (actual_len != chunk * block_size) {
            /* Short read: what was read so far, as a real block device
             * returns on an imperfect disc - unless that is nothing, see the
             * contract above. */
            int short_total = blocks_done + actual_len / block_size;
            if (short_total == 0) {
                /* Both lengths are logged: a few hundred bytes short would be
                 * a drive returning a smaller sector than asked - 2072 rather
                 * than 2352, a raw Mode 2 Form 1 sector without its EDC/ECC -
                 * not a scratch. */
                LOGW("token=%d: read of %d %s at LBA %u returned no usable "
                     "data (%d of %d bytes)",
                     token, num_blocks, what, lba, actual_len,
                     chunk * block_size);
                return (int)LASER_ERR_IO;
            }
            return short_total;
        }

        if (entry != NULL) {
            raise_largest_ok(entry, chunk * block_size);
        }
        blocks_done += chunk;

        if (trial_cap > 0 && blocks_done >= trial_end) {
            lower_transfer_cap(entry, trial_cap, trial_failed);
            trial_cap = 0;
        }
    }

    return blocks_done;
}

static void build_read10_cdb(uint8_t *cdb, uint32_t lba, int blocks,
                             const void *ctx)
{
    (void) ctx;

    cdb[0] = 0x28; /* READ(10) */
    cdb[2] = (uint8_t)(lba >> 24);
    cdb[3] = (uint8_t)(lba >> 16);
    cdb[4] = (uint8_t)(lba >> 8);
    cdb[5] = (uint8_t)(lba);
    cdb[7] = (uint8_t)(blocks >> 8);
    cdb[8] = (uint8_t)(blocks);
}

int laser_read_blocks(int token, uint32_t lba, int num_blocks,
                      uint8_t *buffer)
{
    return read_chunked(token, lba, num_blocks, 2048, 10,
                        build_read10_cdb, NULL, buffer, "blocks", 1);
}

/* Bytes 1 and 9 of READ CD, resolved from the sector kind before the loop
 * starts - see laser_read_cd_blocks() below for what each pairing means. */
typedef struct {
    uint8_t expected_type;
    uint8_t field_flags;
} read_cd_ctx_t;

static void build_read_cd_cdb(uint8_t *cdb, uint32_t lba, int blocks,
                              const void *ctx)
{
    const read_cd_ctx_t *rc = ctx;

    cdb[0] = 0xBE; /* READ CD */
    cdb[1] = rc->expected_type;
    cdb[2] = (uint8_t)(lba >> 24);
    cdb[3] = (uint8_t)(lba >> 16);
    cdb[4] = (uint8_t)(lba >> 8);
    cdb[5] = (uint8_t)(lba);
    cdb[6] = (uint8_t)(blocks >> 16);
    cdb[7] = (uint8_t)(blocks >> 8);
    cdb[8] = (uint8_t)(blocks);
    cdb[9] = rc->field_flags;
}

int laser_read_cd_blocks(int token, uint32_t lba, int num_blocks,
                         laser_cd_sector_t sector_type, uint8_t *buffer)
{
    /* The same for every sector kind - see laser_cd_sector_t in laser.h. */
    const int block_size = 2352;

    /* Byte 1 of the CDB says what the drive should expect to find, byte 9
     * which fields to send back.
     *
     *   AUDIO: Expected Sector Type CD-DA (001b), User Data only. A CD-DA
     *     sector has no sync, header or EDC/ECC - its 2352 bytes are the
     *     user data - and a drive may refuse a request for them with INVALID
     *     FIELD IN CDB, as some do. This is the form CD-DA extractors use. If
     *     a drive ever refuses it, try type "any" (0x00) with User Data only
     *     (0x10).
     *
     *   MODE2_FORM2 and ANY: sync, headers, user data and EDC/ECC (0xF8),
     *     which a Mode 2 sector does have; they differ only in the expected
     *     type, Form 2 (101b) or "all types" (000b).
     *
     *     0xF8 rather than 0xF0, for the LENGTH of the EDC, which no caller
     *     reads:
     *
     *         0xF0, Mode 2 Form 1:  12 + 4 + 8 + 2048        = 2072
     *         0xF0, Mode 2 Form 2:  12 + 4 + 8 + 2324        = 2348
     *         0xF8, Mode 2 Form 1:  12 + 4 + 8 + 2048 + 280  = 2352
     *         0xF8, Mode 2 Form 2:  12 + 4 + 8 + 2324 + 4    = 2352
     *
     *     Only 0xF8 makes both forms 2352 bytes, the stride cdrom.c reads
     *     with. Under 0xF0, a bridge reporting lengths honestly packs Form 2
     *     sectors at 2348, misplacing sector n by 4n bytes and desynchronising
     *     the MPEG stream. */
    uint8_t expected_type;
    uint8_t field_flags;

    switch (sector_type) {
        case LASER_CD_SECTOR_AUDIO:
            expected_type = 0x04;
            field_flags   = 0x10;
            break;

        case LASER_CD_SECTOR_MODE2_FORM2:
            expected_type = 0x14;
            field_flags   = 0xF8;
            break;

        case LASER_CD_SECTOR_ANY:
            expected_type = 0x00;
            field_flags   = 0xF8;
            break;

        default:
            /* A caller bug, rejected before the device is touched, as in
             * laser_scsi_cdb(). */
            LOGE("token=%d: laser_read_cd_blocks: unknown sector type %d",
                 token, (int)sector_type);
            return (int)LASER_ERR_INVALID;
    }

    read_cd_ctx_t ctx = {
        .expected_type = expected_type,
        .field_flags   = field_flags,
    };

    return read_chunked(token, lba, num_blocks, block_size, 12,
                        build_read_cd_cdb, &ctx, buffer, "sectors", 0);
}
