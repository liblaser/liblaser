/*****************************************************************************
 * scsi.c: SCSI-MMC on top of the Bulk-Only transport
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
 * Everything that needs to know what a command MEANS: sense-code
 * classification and the retry policy of laser_scsi_cdb(), the primitive
 * every other command goes through, with the helpers the rest of the layer
 * shares (scsi_internal.h). The rest of the layer:
 *
 *   ready.c   - bringing a drive up: the probes run once at registration
 *               (INQUIRY, TEST UNIT READY) and the wait for its medium
 *   read.c    - the chunked block reads, and what they learn of the disc
 *   medium.c  - the medium: media poll, tray lock, capacity, eject, and the
 *               DVD region check
 *
 * Every transaction goes through laser_bot_send_locked() in bot.c, which
 * knows the wire format and nothing else. Keeping the resilience here spares
 * each consumer - libdvdcss, VLC's cdrom.c, the laser access module - from
 * reimplementing it.
 *****************************************************************************/

#include <stdio.h>
#include <time.h>
#include <unistd.h>

#include "laser.h"
#include "laser_internal.h"
#include "scsi_internal.h"

/* Per-command retry budget: about two and a half seconds of delays, short
 * enough for interactive playback. */
#define LASER_MAX_RETRIES       6
#define LASER_RETRY_DELAY_MS    500

/* For a read - READ(10), READ CD - its first block and block count:
 * for the log, since which sectors failed tells which file, and so which
 * consumer, was reading; and for the throughput sample, which counts only
 * reads following each other. Writes " lba N x M" into [buf], or an empty
 * string for any other command, and the range into [*lba_out, *count_out],
 * left untouched for any other command. Returns whether the command is a
 * read. */
static int describe_read(const uint8_t *cdb, int cdb_len, char *buf, size_t size,
                         uint32_t *lba_out, uint32_t *count_out)
{
    buf[0] = '\0';
    uint32_t count;
    switch (cdb_len >= 10 ? cdb[0] : 0) {
    case 0x28:  /* READ(10) */
        count = (uint32_t)cdb[7] << 8 | cdb[8];
        break;
    case 0xbe:  /* READ CD */
        if (cdb_len < 12)
            return 0;
        count = (uint32_t)cdb[6] << 16 | (uint32_t)cdb[7] << 8 | cdb[8];
        break;
    default:
        return 0;
    }
    const uint32_t lba = (uint32_t)cdb[2] << 24 | (uint32_t)cdb[3] << 16 |
                         (uint32_t)cdb[4] << 8 | cdb[5];
    snprintf(buf, size, " lba %u x %u", (unsigned)lba, (unsigned)count);
    *lba_out = lba;
    *count_out = count;
    return 1;
}

/* REQUEST SENSE (opcode 0x03) - decode the reason for the previous CHECK
 * CONDITION. Caller MUST already hold entry->io_lock. */
static int request_sense_locked(laser_entry_t *entry,
                                uint8_t *sense_key, uint8_t *asc, uint8_t *ascq)
{
    uint8_t cdb[6] = { 0x03, 0x00, 0x00, 0x00, 0x18, 0x00 };
    uint8_t buffer[24] = {0};
    int actual_len = 0;

    if (laser_bot_send_locked(entry, cdb, sizeof(cdb), buffer, sizeof(buffer),
                                1, &actual_len, NULL) < 0) {
        return -1;
    }
    /* The response code says where the three values are, and whose they
     * are (SPC):
     *
     *   70h  fixed format, of the command that just failed
     *   72h  descriptor format, likewise: key, ASC and ASCQ lead it
     *   71h, 73h  DEFERRED: of a command that had already answered - a
     *        cached write, most often. It says nothing of the one that just
     *        failed, and read as its sense it would send the retry policy
     *        after the wrong cause.
     *
     * Any other code is read as fixed format, as every code was before:
     * a bridge that answers the right bytes under a wrong code keeps its
     * sense. On -1 the caller's values are left as they were, "no sense". */
    switch (actual_len >= 1 ? buffer[0] & 0x7f : 0x70) {
    default:
    case 0x70:
        if (actual_len < 14) {
            return -1;
        }
        *sense_key = buffer[2] & 0x0f;
        *asc = buffer[12];
        *ascq = buffer[13];
        return 0;
    case 0x72:
        if (actual_len < 4) {
            return -1;
        }
        *sense_key = buffer[1] & 0x0f;
        *asc = buffer[2];
        *ascq = buffer[3];
        return 0;
    case 0x71:
    case 0x73:
        LOGW("token=%d: deferred sense %02x/%02x/%02x, of an earlier "
             "command: not read as this one's", entry->token,
             (buffer[0] & 0x7f) == 0x71 ? buffer[2] & 0x0f : buffer[1] & 0x0f,
             (buffer[0] & 0x7f) == 0x71 ? buffer[12] : buffer[2],
             (buffer[0] & 0x7f) == 0x71 ? buffer[13] : buffer[3]);
        return -1;
    }
}

/* Whether this command does its work before its status, with no data phase
 * to wait in: see laser_entry_t::slow_status. START STOP UNIT (1Bh) without
 * IMMED - with it, the drive answers at once and works afterwards. */
static int cdb_status_is_slow(const uint8_t *cdb, int cdb_len)
{
    return cdb_len >= 6 && cdb[0] == 0x1b && (cdb[1] & 0x01) == 0;
}

/* See scsi_internal.h. */
int laser_command_locked(laser_entry_t *entry, const uint8_t *cdb,
                         int cdb_len, uint8_t *data, int data_len,
                         int *actual_len, int *csw_status,
                         uint8_t *sense_key, uint8_t *asc, uint8_t *ascq)
{
    int status = -1;
    entry->slow_status = cdb_status_is_slow(cdb, cdb_len);
    int rc = laser_bot_send_locked(entry, cdb, cdb_len, data, data_len,
                                   data_len > 0, actual_len, &status);
    entry->slow_status = 0;
    if (csw_status != NULL)
        *csw_status = status;

    if (sense_key != NULL) {
        *sense_key = 0xff;
        *asc = 0;
        *ascq = 0;
        if (rc != 0 && status == USB_BOT_STATUS_FAIL)
            request_sense_locked(entry, sense_key, asc, ascq);
    }
    return rc;
}

/* See scsi_internal.h. */
int laser_test_unit_ready_locked(laser_entry_t *entry, uint8_t *sense_key,
                                 uint8_t *asc, uint8_t *ascq)
{
    static const uint8_t cdb[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    return laser_command_locked(entry, cdb, sizeof(cdb), NULL, 0, NULL, NULL,
                                sense_key, asc, ascq);
}

/* See scsi_internal.h. */
medium_sense_t laser_medium_sense(uint8_t sense_key, uint8_t asc,
                                  uint8_t ascq)
{
    if (sense_key != SCSI_SENSE_KEY_NOT_READY)
        return MEDIUM_SENSE_OTHER;
    if (asc == SCSI_ASC_MEDIUM_NOT_PRESENT)
        return ascq == SCSI_ASCQ_TRAY_OPEN ? MEDIUM_SENSE_TRAY_OPEN
                                           : MEDIUM_SENSE_ABSENT;
    if (asc == SCSI_ASC_BECOMING_READY) {
        if (ascq == SCSI_ASCQ_NR_START_NEEDED)
            return MEDIUM_SENSE_START_NEEDED;
        if (ascq == SCSI_ASCQ_NR_INTERVENTION)
            return MEDIUM_SENSE_INTERVENTION;
        return MEDIUM_SENSE_LOADING;
    }
    return MEDIUM_SENSE_OTHER;
}

/* See scsi_internal.h. */
void laser_sleep_unlocked(laser_entry_t *entry, int ms)
{
    pthread_mutex_unlock(&entry->io_lock);
    usleep((useconds_t)ms * 1000);
    pthread_mutex_lock(&entry->io_lock);
}

/* The calling thread's abort check (laser_set_abort_cb()). Per thread, so
 * that one consumer giving up stops only its own operations, never those of
 * the other consumers of the token. */
static _Thread_local laser_abort_cb_t abort_cb;
static _Thread_local void *abort_opaque;

void laser_set_abort_cb(laser_abort_cb_t cb, void *opaque)
{
    abort_cb = cb;
    abort_opaque = opaque;
}

/* See scsi_internal.h. */
int laser_must_stop(const laser_entry_t *entry)
{
    return laser_is_cancelled(entry) ||
           (abort_cb != NULL && abort_cb(abort_opaque));
}

/* See scsi_internal.h. */
int64_t laser_monotonic_ms_now(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

/* After a read the drive did not answer, ending in a Reset Recovery: is it
 * the drive, or those sectors? One TEST UNIT READY, under the probe timeouts
 * - a drive able to answer does so at once.
 *
 *   ready            the sectors: the drive answers, and did not answer the
 *                    read. Reading them again would cost the same timeout.
 *   not ready, or    the drive: spinning up after a pause, its medium gone
 *   medium changed   or changed. Nothing is said of the sectors.
 *   no answer        not known: the caller reads once more, as it always
 *                    did.
 *
 * A UNIT ATTENTION other than a changed medium - some bridges raise one
 * after a reset - is cleared by being reported, and says nothing: asked once
 * more.
 *
 * Returns laser_bot_send_locked()'s code, 0 for ready, and the sense as
 * laser_command_locked().
 *
 * Caller MUST already hold entry->io_lock. */
static int unit_ready_after_reset_locked(laser_entry_t *entry,
                                         uint8_t *sense_key, uint8_t *asc,
                                         uint8_t *ascq)
{
    entry->probe_timeouts = 1;
    int rc = laser_test_unit_ready_locked(entry, sense_key, asc, ascq);
    if (rc != 0 && rc != BOT_FAIL_NO_DEVICE &&
        *sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION &&
        !(*asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
          *ascq == SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED)) {
        rc = laser_test_unit_ready_locked(entry, sense_key, asc, ascq);
    }
    entry->probe_timeouts = 0;
    return rc;
}

/* Only transfers at least this large are sampled: smaller ones measure
 * command latency, not the link. And only reads continuing the previous one:
 * see the caller. */
#define LASER_XFER_SAMPLE_MIN_BYTES  (32 * 1024)

/* Bytes sampled before the average is believed: one slow transfer - a seek,
 * a retried stall - proves nothing. */
#define LASER_XFER_SAMPLE_BYTES      (1024 * 1024)

/* The rate, in KiB/s, below which a link of this wMaxPacketSize is not
 * performing as its speed implies. Far below what it can do, to catch only a
 * drive running at a fraction of its capability: healthy high speed sustains
 * about 15,000 KiB/s, the underpowered drive this was written for 90. Full
 * speed gets a lower floor, ~1,000 KiB/s being its ceiling. */
static uint32_t slow_floor_kib_per_s(uint16_t max_packet)
{
    return max_packet <= 64 ? 250 : 1000;
}

/* Accumulates read throughput, and reports it once per registration - as a
 * warning if it is far below what the link should carry.
 *
 * An underpowered drive enumerates, answers every command correctly and
 * reads at a fraction of its rate. Nothing fails, so nothing else here would
 * report it; the only visible symptom is video arriving late, which points
 * everywhere but at the cause. The healthy figure is logged too, for bug
 * reports.
 *
 * Caller MUST already hold entry->io_lock. */
static void sample_throughput_locked(laser_entry_t *entry, int data_len,
                                     const struct timespec *started)
{
    if (entry->slow_warned || data_len < LASER_XFER_SAMPLE_MIN_BYTES) {
        return;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    const int64_t us = (int64_t)(now.tv_sec - started->tv_sec) * 1000000
                     + (now.tv_nsec - started->tv_nsec) / 1000;

    entry->xfer_bytes += (uint64_t)data_len;
    entry->xfer_us    += (uint64_t)(us > 0 ? us : 0);
    entry->xfer_count++;

    if (entry->xfer_bytes < LASER_XFER_SAMPLE_BYTES || entry->xfer_us == 0) {
        return;
    }

    const uint64_t kib_per_s = (entry->xfer_bytes * 1000000)
                             / (entry->xfer_us * 1024);
    const uint32_t floor_kib = slow_floor_kib_per_s(entry->ep_max_packet);

    /* Per transfer, which is the shape a reader can compare by eye. */
    const uint64_t per_kib = entry->xfer_bytes / entry->xfer_count / 1024;
    const uint64_t per_ms  = entry->xfer_us / entry->xfer_count / 1000;

    entry->slow_warned = 1;

    if (kib_per_s >= floor_kib) {
        LOGI("token=%d: sustained read throughput %llu KiB/s "
             "(%llu KiB in %llu ms per transfer, wMaxPacketSize %u)",
             entry->token, (unsigned long long)kib_per_s,
             (unsigned long long)per_kib, (unsigned long long)per_ms,
             entry->ep_max_packet);
        return;
    }

    /* Kept under the log line limit (LOG_LINE_MAX in registry.c): the advice
     * at its end is the point of it. */
    LOGW("token=%d, usb %04x:%04x: read throughput only %llu KiB/s "
         "(%llu KiB in %llu ms per transfer, wMaxPacketSize %u) with no "
         "command failing - most likely an underpowered drive: try a powered "
         "hub or a stronger supply",
         entry->token, entry->vid, entry->pid,
         (unsigned long long)kib_per_s, (unsigned long long)per_kib,
         (unsigned long long)per_ms, entry->ep_max_packet);
}

/* ============================================================================
 * Public: laser_scsi_cdb() - the generic, retrying, mutex-protected CDB
 * primitive everything else goes through.
 * ============================================================================ */

/* See scsi_internal.h. */
void laser_retry_delay(laser_entry_t *entry, int idempotent)
{
    /* An attempt failed: polls stay away while the command retries. */
    entry->last_failure_ms = laser_monotonic_ms_now();

    if (idempotent)
        laser_sleep_unlocked(entry, LASER_RETRY_DELAY_MS);
    else
        usleep(LASER_RETRY_DELAY_MS * 1000);
}

laser_status_t laser_scsi_cdb(int token,
                              const uint8_t *cdb, int cdb_len,
                              uint8_t *data, int data_len,
                              int data_in, int *actual_len)
{
    return laser_scsi_cdb_read(token, cdb, cdb_len, data, data_len, data_in,
                               actual_len, LASER_READ_MAX_MEDIUM_ERRORS,
                               LASER_READ_MAX_RESETS, NULL);
}

/* See scsi_internal.h. */
laser_status_t laser_scsi_cdb_read(int token,
                                   const uint8_t *cdb, int cdb_len,
                                   uint8_t *data, int data_len,
                                   int data_in, int *actual_len,
                                   int max_medium_errors, int max_resets,
                                   laser_failure_t *why)
{
    if (why != NULL) {
        why->cause = LASER_FAILURE_OTHER;
        why->sense_key = 0xff;
        why->asc = 0;
        why->ascq = 0;
    }

    /* Caller bugs, caught here because their symptoms appear far from the
     * cause: an oversized cdb_len overflows the CBW's 16-byte field, and a
     * NULL buffer with a data length announces a data phase that never
     * happens, ending in a Reset Recovery with nothing in the log pointing
     * at the mistake. */
    if (cdb == NULL || cdb_len <= 0 || cdb_len > 16) {
        LOGE("token=%d: invalid CDB (%p, %d bytes; expected 1-16)",
             token, (const void *)cdb, cdb_len);
        return LASER_ERR_INVALID;
    }
    if (data_len < 0 || (data == NULL && data_len > 0)) {
        LOGE("token=%d: invalid data phase (%p, %d bytes)",
             token, (const void *)data, data_len);
        return LASER_ERR_INVALID;
    }
    if (data_len > LASER_MAX_BYTES_PER_TRANSFER) {
        /* Warned about, not rejected: libusb may well carry it, but it is
         * beyond what is reliable across Android host controllers, and the
         * block helpers exist so that nobody has to. */
        LOGW("token=%d: %d-byte data phase exceeds the %d-byte per-transfer "
             "budget; prefer laser_read_blocks()/read_cd_blocks()",
             token, data_len, LASER_MAX_BYTES_PER_TRANSFER);
    }

    /* A lookup, never a registration: a command on an unclaimed token is a
     * caller bug, and registering here would create an entry nobody
     * releases. Pinned until every return below. */
    laser_entry_t *entry = laser_pin(token);
    if (entry == NULL) {
        LOGW("token=%d: no such token - is a claim held? "
             "(laser_acquire() is what registers a device)", token);
        return LASER_ERR_NO_SUCH_TOKEN;
    }

    /* Whether this command changes CSS authentication state decides two
     * things: whether a session is required, and whether a retry is safe. */
    const int css_state_changing = laser_cdb_changes_css_state(cdb, cdb_len);

    /* Refused outside any session rather than wrapped in one: exclusion over
     * a single transaction is io_lock already, and what needs protecting is
     * the sequence, which only the consumer can see. This stops an
     * undeclared consumer from taking an AGID mid-handshake.
     *
     * Checked before io_lock - see the lock order in registry.c. */
    if (css_state_changing && !laser_css_session_is_open(entry)) {
        LOGE("token=%d, usb %04x:%04x: CSS command 0x%02x issued with no "
             "session open - refused. Call laser_css_session_begin() "
             "first; see the session contract in laser.h",
             token, entry->vid, entry->pid, cdb[0]);
        laser_unpin(entry);
        return LASER_ERR_INVALID;
    }

    /* Before queueing on io_lock, so that a command arriving during teardown
     * does not first wait out the transaction in progress. */
    if (laser_must_stop(entry)) {
        laser_unpin(entry);
        return LASER_ERR_CANCELLED;
    }

    if (entry->device_gone) {
        laser_unpin(entry);
        return LASER_ERR_NO_DEVICE;
    }

    pthread_mutex_lock(&entry->io_lock);

    laser_status_t result = LASER_ERR_IO;

    /* Is this command safe to send twice? Reads are. A command changing CSS
     * state is not: each accepted handshake step advances the drive's state
     * machine, and every accepted AGID request takes one of the drive's four
     * AGIDs - a retry through a UNIT ATTENTION could exhaust them until the
     * drive is unplugged. Recovering such a failure - invalidating the AGID
     * and starting over - is libdvdcss's job.
     *
     * Not keyed on direction: three handshake steps are REPORT KEY, which is
     * DATA-IN. And keyed on the same predicate as the session check, so that
     * the read-only key queries libdvdcss uses to decide whether a disc is
     * scrambled at all keep their retries. */
    const int idempotent = !css_state_changing && (data_in || data_len == 0);

    /* Last sense seen, so that the final failure can say why. */
    uint8_t last_sense_key = 0xff, last_asc = 0, last_ascq = 0;

    /* Attempts made, and whether a branch below already explained the
     * failure: the summary at the end must not claim a budget was exhausted
     * when the loop broke out on the first refusal. */
    int attempts_made = 0;
    int reason_logged = 0;

    /* Attempts that failed before the CBW could be handed over - see the
     * verdict after the loop. */
    int cbw_never_sent = 0;

    /* Set by the first 04h/01h BECOMING READY: see its branch below. */
    int becoming_ready = 0;
    int64_t becoming_ready_since = 0;

    /* Which sectors a read asks for, appended to its messages; and its
     * Reset Recoveries in a row, see LASER_READ_MAX_RESETS. */
    char where[40];
    uint32_t read_lba = 0, read_count = 0;
    const int is_read = describe_read(cdb, cdb_len, where, sizeof(where),
                                      &read_lba, &read_count);
    int resets_in_a_row = 0;
    int medium_errors = 0;

    /* Whether the drive, stopped, has been told to start: once a command. */
    int start_sent = 0;

    for (int attempt = 1; attempt <= LASER_MAX_RETRIES; attempt++) {
        attempts_made = attempt;

        /* Cancellation is honoured between attempts: a transfer already
         * handed to the kernel runs to its timeout, but no further attempt
         * follows. So is an abort, the "becoming ready" wait included: each
         * of its rounds is an attempt. */
        if (laser_must_stop(entry)) {
            LOGI("token=%d: cancelled, abandoning cdb 0x%02x after %d "
                 "attempt(s)", token, cdb[0], attempt - 1);
            result = LASER_ERR_CANCELLED;
            break;
        }

        /* Local to this attempt: request_sense_locked() below issues its
         * own BOT transaction, and this value must survive it intact. */
        int csw_status = -1;
        struct timespec xfer_started;
        clock_gettime(CLOCK_MONOTONIC, &xfer_started);

        entry->slow_status = cdb_status_is_slow(cdb, cdb_len);
        int rc = laser_bot_send_locked(entry, cdb, cdb_len, data, data_len,
                                         data_in, actual_len, &csw_status);
        entry->slow_status = 0;
        if (rc == 0) {
            /* A success after retries is logged: otherwise a drive limping
             * through its budget on every command looks, in the log, like
             * one answering at once. */
            if (attempt > 1) {
                LOGW("token=%d: cdb 0x%02x%s succeeded on attempt %d/%d - "
                     "earlier attempts failed and were retried",
                     token, cdb[0], where, attempt, LASER_MAX_RETRIES);
            }
            /* Only a read following the previous one is a sample: a seek
             * costs what a slow link does, and a classification - the UDF
             * walk, the IFOs - is mostly seeks. */
            if (is_read) {
                if (read_lba == entry->xfer_next_lba)
                    sample_throughput_locked(entry, data_len, &xfer_started);
                entry->xfer_next_lba = read_lba + read_count;
            }
            result = LASER_OK;
            break;
        }

        if (rc == BOT_FAIL_NOT_SENT) {
            cbw_never_sent++;
        }

        if (rc == BOT_FAIL_NO_DEVICE) {
            /* No retry can bring the drive back, and each would cost a full
             * set of USB timeouts. Reported as its own status: an I/O error
             * would invite retries, and MEDIA_GONE would say the drive
             * answered. */
            LOGW("token=%d: device no longer present, not retrying", token);
            entry->device_gone = 1;
            result = LASER_ERR_NO_DEVICE;
            break;
        }

        uint8_t sense_key = 0xff, asc = 0, ascq = 0;
        if (rc == BOT_FAIL_PHASE_ERROR) {
            /* The device was just reset: there is no completed command left
             * for REQUEST SENSE to explain. The attempt still counts. */
            LOGW("token=%d: cdb 0x%02x%s ended in a Reset Recovery (attempt "
                 "%d/%d)", token, cdb[0], where, attempt, LASER_MAX_RETRIES);
            if (is_read) {
                resets_in_a_row++;

                /* A read made ahead: nobody waits for these sectors, and
                 * nothing will be held against them. Not asked which is to
                 * blame, not read again. */
                if (max_resets < LASER_READ_MAX_RESETS) {
                    LOGI("token=%d: cdb 0x%02x%s - not answered, and read "
                         "ahead: giving up on this read",
                         token, cdb[0], where);
                    reason_logged = 1;
                    result = LASER_ERR_IO;
                    break;
                }

                /* The drive, or these sectors? */
                int ready = unit_ready_after_reset_locked(entry, &sense_key,
                                                          &asc, &ascq);
                if (ready == BOT_FAIL_NO_DEVICE) {
                    LOGW("token=%d: device no longer present, not retrying",
                         token);
                    entry->device_gone = 1;
                    result = LASER_ERR_NO_DEVICE;
                    break;
                }
                if (sense_key != 0xff) {
                    last_sense_key = sense_key;
                    last_asc = asc;
                    last_ascq = ascq;
                }

                const medium_sense_t state =
                    laser_medium_sense(sense_key, asc, ascq);
                if (ready == 0) {
                    LOGW("token=%d: cdb 0x%02x%s - the drive did not answer "
                         "the read and answers that it is ready: these "
                         "sectors, giving up on this read",
                         token, cdb[0], where);
                    reason_logged = 1;
                    result = LASER_ERR_IO;
                    if (why != NULL) {
                        why->cause = LASER_FAILURE_NO_ANSWER;
                    }
                    break;
                }
                if (state == MEDIUM_SENSE_LOADING) {
                    /* Spinning up: waited for below, and the read is not
                     * held against the sectors. */
                    resets_in_a_row = 0;
                } else if (state == MEDIUM_SENSE_OTHER &&
                           !(asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
                             ascq == SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED) &&
                           resets_in_a_row >= max_resets) {
                    LOGW("token=%d: cdb 0x%02x%s - the drive did not answer "
                         "%d times in a row (sense %02x/%02x/%02x), giving "
                         "up on this read", token, cdb[0], where,
                         resets_in_a_row, sense_key, asc, ascq);
                    reason_logged = 1;
                    result = LASER_ERR_IO;
                    /* Stuck on the sectors if it answered nothing since;
                     * a drive saying why it is not ready is not. */
                    if (why != NULL && sense_key == 0xff) {
                        why->cause = LASER_FAILURE_NO_ANSWER;
                    }
                    break;
                }
            }
        } else if (csw_status == USB_BOT_STATUS_FAIL) {
            request_sense_locked(entry, &sense_key, &asc, &ascq);
            last_sense_key = sense_key;
            last_asc = asc;
            last_ascq = ascq;
        }

        if (rc != BOT_FAIL_PHASE_ERROR)
            resets_in_a_row = 0;

        /* Disc gone or swapped: never retried, so that an ejection is felt at
         * once. Every 3Ah qualifier, unlike in the spin-up wait: that wait
         * has already run, so a fresh 3Ah here means the disc really is
         * gone. */
        const medium_sense_t medium = laser_medium_sense(sense_key, asc, ascq);
        if (medium == MEDIUM_SENSE_ABSENT || medium == MEDIUM_SENSE_TRAY_OPEN) {
            LOGW("token=%d: no disc present (%02x/%02x/%02x), not retrying",
                 token, sense_key, asc, ascq);
            laser_disc_changed_locked(entry);
            result = LASER_ERR_MEDIA_GONE;
            break;
        }
        if (asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
            ascq == SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED) {
            /* Right after the drive said it was becoming ready - to this
             * command, or to a spin-up wait that gave up - this is the NOT
             * READY TO READY change that ends a spin-up, not a disc swap.
             * Retried at once, without spending an attempt; once only, since
             * both flags are cleared. */
            if (becoming_ready || entry->wait_state != LASER_WAIT_READY) {
                LOGI("token=%d: drive now ready (06/28/00 after spin-up), "
                     "retrying cdb 0x%02x", token, cdb[0]);
                becoming_ready = 0;
                entry->wait_state = LASER_WAIT_READY;
                attempt--;
                continue;
            }
            LOGW("token=%d: medium may have changed, not retrying", token);
            laser_disc_changed_locked(entry);
            result = LASER_ERR_MEDIA_GONE;
            break;
        }
        /* Copy-protection refusals, told apart before the generic refusal
         * below swallows them. None is retried, but they mean different
         * things to the caller, who cannot recover the qualifier once this
         * function has returned. */
        if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST &&
            asc == SCSI_ASC_COPY_PROTECTION) {
            switch (ascq) {
            case SCSI_ASCQ_CP_SCRAMBLED:
                LOGW("token=%d: cdb 0x%02x%s refused, scrambled sector without "
                     "authentication (sense %02x/%02x/%02x)",
                     token, cdb[0], where, sense_key, asc, ascq);
                result = LASER_ERR_SCRAMBLED;
                break;
            case SCSI_ASCQ_CP_AUTH_FAILURE:
            case SCSI_ASCQ_CP_KEY_NOT_PRESENT:
            case SCSI_ASCQ_CP_KEY_NOT_ESTABLISHED:
                LOGW("token=%d: cdb 0x%02x refused, no CSS session in force "
                     "(sense %02x/%02x/%02x) - re-authentication may make "
                     "this succeed", token, cdb[0], sense_key, asc, ascq);
                result = LASER_ERR_NO_KEY;
                break;
            case SCSI_ASCQ_CP_REGION_MISMATCH:
            case SCSI_ASCQ_CP_REGION_PERMANENT:
                LOGW("token=%d, usb %04x:%04x: cdb 0x%02x refused on REGION "
                     "(sense %02x/%02x/%02x) - the disc's region does not "
                     "match the drive's; this is not an authentication "
                     "problem and no amount of key exchange will fix it",
                     token, entry->vid, entry->pid, cdb[0],
                     sense_key, asc, ascq);
                result = LASER_ERR_REGION;
                break;
            default:
                LOGW("token=%d: cdb 0x%02x refused, unassigned copy-protection "
                     "qualifier (sense %02x/%02x/%02x)",
                     token, cdb[0], sense_key, asc, ascq);
                result = LASER_ERR_REFUSED;
                break;
            }
            reason_logged = 1;
            break;
        }

        /* Permanently refused: the drive will answer the same way every
         * time, and six delayed retries per read would turn a clean error
         * into what a user experiences as a hang. BLANK CHECK with them:
         * nothing is recorded where the command reads, the area of a
         * recordable disc past what was written, and nothing will be. */
        if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST ||
            sense_key == SCSI_SENSE_KEY_DATA_PROTECT ||
            sense_key == SCSI_SENSE_KEY_BLANK_CHECK) {
            LOGW("token=%d: cdb 0x%02x%s permanently refused "
                 "(sense %02x/%02x/%02x), not retrying",
                 token, cdb[0], where, sense_key, asc, ascq);
            reason_logged = 1;
            result = LASER_ERR_REFUSED;
            break;
        }

        /* An unreadable sector: see LASER_READ_MAX_MEDIUM_ERRORS. The same
         * budget for a HARDWARE ERROR, which the drive answers after trying
         * as hard - but only a medium error is the disc's, for the caller
         * to hold against the blocks. */
        if (is_read && (sense_key == SCSI_SENSE_KEY_MEDIUM_ERROR ||
                        sense_key == SCSI_SENSE_KEY_HARDWARE_ERROR) &&
            ++medium_errors >= max_medium_errors) {
            LOGW("token=%d: cdb 0x%02x%s unreadable %d times "
                 "(sense %02x/%02x/%02x), giving up on this read",
                 token, cdb[0], where, medium_errors, sense_key, asc, ascq);
            reason_logged = 1;
            result = LASER_ERR_IO;
            if (why != NULL && sense_key == SCSI_SENSE_KEY_MEDIUM_ERROR) {
                why->cause = LASER_FAILURE_MEDIUM;
            }
            break;
        }

        /* A non-idempotent command the drive may have acted on: stop here,
         * unless its CBW never went out. Only here, after the sense has been
         * read and classified above: REQUEST SENSE replays nothing, and
         * without it a refused key command - a region mismatch during the
         * handshake, a disc gone - reached libdvdcss as a bare I/O error. */
        if (!idempotent && rc != BOT_FAIL_NOT_SENT) {
            LOGW("token=%d: key command 0x%02x failed after its CBW was sent "
                 "(sense %02x/%02x/%02x), not retrying - the drive's state may "
                 "have advanced", token, cdb[0], sense_key, asc, ascq);
            reason_logged = 1;
            result = LASER_ERR_IO;
            break;
        }

        /* Not ready until somebody goes to the drive: waiting thirty
         * seconds, as for one becoming ready, brings nothing. */
        if (medium == MEDIUM_SENSE_INTERVENTION) {
            LOGW("token=%d: cdb 0x%02x%s - the drive asks for manual "
                 "intervention (sense %02x/%02x/%02x), not retrying",
                 token, cdb[0], where, sense_key, asc, ascq);
            reason_logged = 1;
            result = LASER_ERR_IO;
            break;
        }

        /* Stopped, and saying it will stay so until told to start: told,
         * once, without waiting for the spin-up - the wait below does, the
         * drive answering "becoming ready" from then on. Without spending
         * an attempt. A second such answer gets the ordinary delayed
         * retry. */
        if (medium == MEDIUM_SENSE_START_NEEDED && !start_sent) {
            static const uint8_t start_unit[6] = {
                0x1b, 0x01 /* IMMED */, 0x00, 0x00, 0x01 /* START */, 0x00,
            };
            LOGI("token=%d: cdb 0x%02x: drive stopped (sense %02x/%02x/%02x),"
                 " starting it", token, cdb[0], sense_key, asc, ascq);
            start_sent = 1;
            laser_command_locked(entry, start_unit, sizeof(start_unit), NULL,
                                 0, NULL, NULL, NULL, NULL, NULL);
            attempt--;
            laser_retry_delay(entry, idempotent);
            continue;
        }

        /* A drive waking from standby answers "becoming ready" far longer
         * than LASER_MAX_RETRIES covers: a Blu-ray drive idle for a few
         * minutes took 45 s, past the spin-up wait at registration. Waited
         * out under LASER_BECOMING_READY_MAX_WALL_MS, without spending
         * attempts; cancellation is still checked on every round. */
        if (medium == MEDIUM_SENSE_LOADING) {
            if (!becoming_ready) {
                becoming_ready = 1;
                becoming_ready_since = laser_monotonic_ms_now();
                LOGI("token=%d: cdb 0x%02x: drive becoming ready, waiting up "
                     "to %dms", token, cdb[0],
                     LASER_BECOMING_READY_MAX_WALL_MS);
            }
            if (laser_monotonic_ms_now() - becoming_ready_since <
                LASER_BECOMING_READY_MAX_WALL_MS) {
                attempt--;
                laser_retry_delay(entry, idempotent);
                continue;
            }
        }

        if (asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
            ascq == SCSI_ASCQ_FORMAT_LAYER_CHANGED) {
            /* A dual-layer DVD crossing layers, around the middle of a film:
             * the read that follows normally succeeds, so the ordinary
             * delayed retry below applies. */
            LOGW("token=%d: format layer changed (dual-layer boundary), "
                 "retrying", token);
        }

        /* UNIT ATTENTION clears itself by being reported: retry at once.
         * Anything else - not ready, a transport error, an unexpected sense -
         * gets the delayed retry. */
        if (sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION) {
            continue;
        }

        if (attempt < LASER_MAX_RETRIES) {
            laser_retry_delay(entry, idempotent);
        }
    }

    /* Every attempt failed before the CBW left: the device is gone, whatever
     * libusb called it. A descriptor closed under the handle, or a controller
     * tearing the URB down before marking the device absent, gives
     * LIBUSB_ERROR_IO rather than NO_DEVICE, which one attempt cannot tell
     * from a bridge having a bad moment - but a device that will not take 31
     * bytes on any of LASER_MAX_RETRIES attempts is not there. Never over a
     * cancellation, which the caller asked for. */
    if (result != LASER_OK && result != LASER_ERR_CANCELLED &&
        attempts_made > 0 && cbw_never_sent == attempts_made) {
        LOGW("token=%d: cdb 0x%02x - the CBW could not be sent on any of "
             "%d attempt(s); treating the device as gone",
             token, cdb[0], attempts_made);
        entry->device_gone = 1;
        result = LASER_ERR_NO_DEVICE;
        reason_logged = 1;
    }

    /* Every way out of the loop sets a status of laser.h, LASER_ERR_IO by
     * default: the budget spent. Summed up here, unless a branch above has
     * already said why - which the non-idempotent path always has. */
    if (result == LASER_ERR_IO) {
        if (idempotent && !reason_logged) {
            LOGW("token=%d: cdb 0x%02x%s failed after %d attempt(s) "
                 "(last sense %02x/%02x/%02x)",
                 token, cdb[0], where, attempts_made,
                 last_sense_key, last_asc, last_ascq);
        }

        /* The drive failed the command outright - not a refusal of one
         * sector or key, answered at once: polls give it time to recover. */
        entry->last_failure_ms = laser_monotonic_ms_now();
    }

    if (why != NULL) {
        why->sense_key = last_sense_key;
        why->asc = last_asc;
        why->ascq = last_ascq;
    }

    pthread_mutex_unlock(&entry->io_lock);
    laser_unpin(entry);
    return result;
}

int laser_status_is_positional(int status)
{
    /* In the file that produces these values, so that whoever adds a status
     * finds this list beside it. */
    switch (status) {
    case LASER_ERR_SCRAMBLED:
    case LASER_ERR_REGION:
    case LASER_ERR_REFUSED:
        return 1;
    default:
        /* Including every non-negative value: a block count is a successful
         * read, not a status. */
        return 0;
    }
}
