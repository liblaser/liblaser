/*****************************************************************************
 * medium.c: the medium
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
 * What the drive says of the disc in its tray: the media poll (GET EVENT
 * STATUS NOTIFICATION, or TEST UNIT READY on a drive without it), the tray
 * lock, eject, the capacity, and whether the disc's DVD region matches the
 * drive's.
 *****************************************************************************/

#include <string.h>

#include "laser.h"
#include "laser_internal.h"
#include "scsi_internal.h"

/* How long after a failed attempt a media poll leaves the drive alone (see
 * poll_lock()): past the retry delay, so that a poll never lands between
 * two attempts, and past one poll interval of the application's, so that
 * the next poll finds a drive that has had time to recover. */
#define LASER_POLL_QUIET_MS     3000

/* See scsi_internal.h. */
int laser_gesn_media_locked(laser_entry_t *entry, gesn_media_t *out,
                            uint8_t *sense_key, uint8_t *asc, uint8_t *ascq)
{
    uint8_t cdb[10] = { 0 };
    cdb[0] = 0x4a;                  /* GET EVENT STATUS NOTIFICATION */
    cdb[1] = 0x01;                  /* Polled */
    cdb[4] = 1 << GESN_CLASS_MEDIA; /* notification class request */
    cdb[8] = 0x08;                  /* header + media event descriptor */

    uint8_t buf[8] = { 0 };
    memset(out, 0, sizeof(*out));
    int rc = laser_command_locked(entry, cdb, sizeof(cdb), buf, sizeof(buf),
                                  &out->length, NULL, sense_key, asc, ascq);
    if (rc != 0)
        return rc;
    if (out->length < 8)
        return -1;

    /* Header: [0..1] descriptor length, [2] bit 7 NEA and the class in bits
     * 0-2, [3] supported classes. Media descriptor: [4] event code, [5]
     * status bits, [6..7] slot range. */
    out->nea       = (buf[2] & 0x80) != 0;
    out->cls       = buf[2] & 0x07;
    out->event     = buf[4] & 0x0f;
    out->tray_open = (buf[5] & GESN_MEDIA_STATUS_TRAY_OPEN) != 0;
    out->present   = (buf[5] & GESN_MEDIA_STATUS_PRESENT) != 0;
    memcpy(out->raw, buf + 2, sizeof(out->raw));
    return 0;
}

/* ============================================================================
 * DVD region (laser_region_mismatch)
 * ========================================================================= */

/* Region masks, in both structures, are "one bit per region, SET means
 * PROHIBITED": a region-1 disc reads 0xFE, a drive set to region 2 reads
 * 0xFD. So the regions something permits are the complement, and disc and
 * drive agree exactly when their permitted sets intersect. */
#define LASER_REGION_ALL        0xFF

/* Byte 4 >> 6 of the RPC state. 0: no region set yet, which is no mismatch -
 * the drive adopts the first disc's region by itself. */
#define LASER_RPC_TYPE_NONE     0x00

/* Byte 6 of the RPC state. 0 is RPC-1: the drive enforces nothing. */
#define LASER_RPC_SCHEME_NONE   0x00

/* REPORT KEY, key format 08h: the drive's RPC state.
 *
 * @return 1 and the three fields set, 0 if the drive would not answer -
 *         which includes every non-DVD drive and is not an error here. */
static int report_rpc_state(int token, uint8_t *type, uint8_t *mask,
                            uint8_t *scheme)
{
    uint8_t cdb[12] = { 0 };
    uint8_t data[8] = { 0 };
    int actual = 0;

    cdb[0]  = 0xA4;                  /* REPORT KEY */
    cdb[8]  = sizeof(data) >> 8;     /* allocation length, big-endian */
    cdb[9]  = sizeof(data) & 0xFF;
    cdb[10] = 0x08;                  /* key format: RPC state. No AGID for
                                      * this format: the drive allocates
                                      * nothing. */

    if (laser_scsi_cdb(token, cdb, sizeof(cdb), data, sizeof(data), 1,
                       &actual) != LASER_OK
     || actual < (int)sizeof(data)) {
        return 0;
    }

    *type   = data[4] >> 6;
    *mask   = data[5];
    *scheme = data[6];
    return 1;
}

/* READ DVD STRUCTURE, format 01h: the disc's copyright information, whose
 * second byte is the Region Management Information.
 *
 * @return 1 and the mask set, 0 if the medium is not a DVD or carries no
 *         copyright structure. */
static int read_disc_region_mask(int token, uint8_t *rmi)
{
    uint8_t cdb[12] = { 0 };
    uint8_t data[8] = { 0 };
    int actual = 0;

    cdb[0] = 0xAD;                   /* READ DVD STRUCTURE */
    cdb[6] = 0x00;                   /* layer 0 */
    cdb[7] = 0x01;                   /* format: copyright information */
    cdb[8] = sizeof(data) >> 8;
    cdb[9] = sizeof(data) & 0xFF;

    if (laser_scsi_cdb(token, cdb, sizeof(cdb), data, sizeof(data), 1,
                       &actual) != LASER_OK
     || actual < (int)sizeof(data)) {
        return 0;
    }

    *rmi = data[5];
    return 1;
}

int laser_region_mismatch(int token, uint8_t *drive_mask, uint8_t *disc_mask)
{
    uint8_t type, dr_mask, scheme, di_mask;

    if (!report_rpc_state(token, &type, &dr_mask, &scheme)) {
        return 0;                    /* not a DVD drive, or would not say */
    }

    if (scheme == LASER_RPC_SCHEME_NONE || type == LASER_RPC_TYPE_NONE) {
        return 0;                    /* enforces nothing, or nothing set yet */
    }

    if (!read_disc_region_mask(token, &di_mask)) {
        return 0;                    /* not a DVD, or no copyright structure */
    }

    if (di_mask == 0x00) {
        return 0;                    /* region-free disc: plays anywhere */
    }

    /* Permitted sets are the complements; they agree if they intersect. */
    if ((uint8_t)(~dr_mask & ~di_mask & LASER_REGION_ALL) != 0) {
        return 0;
    }

    /* Written only on the answer that makes them meaningful - see laser.h. */
    if (drive_mask != NULL) {
        *drive_mask = dr_mask;
    }
    if (disc_mask != NULL) {
        *disc_mask = di_mask;
    }

    LOGI("token=%d: region mismatch, drive mask 0x%02x, disc mask 0x%02x",
         token, dr_mask, di_mask);
    return 1;
}

/* ============================================================================
 * The medium: events, tray lock, eject - see laser.h
 * ========================================================================= */

/* Takes io_lock for a poll, or fails at once - the poll skipped - if another
 * command holds it, or if a command failed an attempt less than
 * LASER_POLL_QUIET_MS ago. A poll only matters when the drive is idle, and
 * the next one comes a second or two later. Queued behind a read the drive
 * is failing, or slipping in while that read waits between two attempts -
 * io_lock is free then (laser_retry_delay()) - it sent its own command to a
 * drive still stuck, timed out, and added a Reset Recovery between the read's
 * attempts. Nothing is lost by skipping: the drive keeps a media event, and
 * a UNIT ATTENTION, until it is reported. */
static int poll_lock(laser_entry_t *entry)
{
    if (pthread_mutex_trylock(&entry->io_lock) != 0)
        return 0;

    if (entry->last_failure_ms != 0 &&
        laser_monotonic_ms_now() - entry->last_failure_ms
            < LASER_POLL_QUIET_MS) {
        pthread_mutex_unlock(&entry->io_lock);
        return 0;
    }
    return 1;
}

/* One GET EVENT STATUS NOTIFICATION, polled, media class.
 *
 * A single attempt, taken directly rather than through laser_scsi_cdb(): a
 * poll comes back every second or two, and the read retry policy - six
 * attempts, each possibly ending in a Reset Recovery - held the drive for
 * fifteen seconds and more on a failing one, the player's reads waiting
 * behind it. A UNIT ATTENTION alone is asked again at once: reported, it is
 * cleared, and costs nothing.
 *
 * Returns LASER_OK and fills @p out; LASER_ERR_REFUSED only when the drive
 * says it has no media events - the command refused as ILLEGAL REQUEST, or
 * answered with "no event class supported" - which turns them off for the
 * token; LASER_ERR_NO_DEVICE; LASER_ERR_IO for anything else, a short or
 * odd answer included: one of those from a bridge having a bad moment must
 * not cost the token its media events, and with them the eject button of a
 * locked tray, for good. */
static laser_status_t media_events(laser_entry_t *entry,
                                   laser_media_status_t *out)
{
    gesn_media_t g;
    int rc = -1;
    uint8_t sense_key = 0xff, asc = 0, ascq = 0;

    /* A drive busy with another consumer's command is not polled: see
     * poll_lock(). */
    if (!poll_lock(entry))
        return LASER_ERR_IO;
    for (int attempt = 1; attempt <= 2; attempt++) {
        rc = laser_gesn_media_locked(entry, &g, &sense_key, &asc, &ascq);
        if (rc == 0 || rc == BOT_FAIL_NO_DEVICE ||
            sense_key != SCSI_SENSE_KEY_UNIT_ATTENTION)
            break;
        /* The UNIT ATTENTION of a changed medium, consumed here: said to
         * the other consumers, whatever event follows. */
        if (asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
            ascq == SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED)
            laser_disc_changed_locked(entry);
    }
    pthread_mutex_unlock(&entry->io_lock);

    if (rc == BOT_FAIL_NO_DEVICE) {
        entry->device_gone = 1;
        return LASER_ERR_NO_DEVICE;
    }
    if (rc != 0) {
        if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST)
            return LASER_ERR_REFUSED;
        return LASER_ERR_IO;
    }

    if (g.nea)
        return LASER_ERR_REFUSED;
    if (g.cls != GESN_CLASS_MEDIA)
        return LASER_ERR_IO;

    switch (g.event) {
    case GESN_MEDIA_EJECT_REQUEST:
        out->event = LASER_MEDIA_EJECT_REQUEST;
        break;
    case GESN_MEDIA_NEW:
    case GESN_MEDIA_CHANGED:
        out->event = LASER_MEDIA_NEW;
        break;
    case GESN_MEDIA_REMOVAL:
        out->event = LASER_MEDIA_REMOVED;
        break;
    default:
        out->event = LASER_MEDIA_NO_EVENT;
        break;
    }
    out->tray_open = g.tray_open;
    /* The "present" bit is not trusted: see laser_medium_t. */
    out->medium = LASER_MEDIUM_UNKNOWN;

    if (out->event != LASER_MEDIA_NO_EVENT)
        LOGI("token=%d: media event %d (raw %02x %02x), tray %s", entry->token,
             (int)out->event, g.raw[2], g.raw[3],
             out->tray_open ? "open" : "closed");
    return LASER_OK;
}

/* The same, from TEST UNIT READY, for a drive without media events. Taken
 * directly rather than through laser_scsi_cdb(), whose retries would wait
 * out the very states reported here. */
static laser_status_t media_from_unit_ready(laser_entry_t *entry,
                                            laser_media_status_t *out)
{
    if (!poll_lock(entry))
        return LASER_ERR_IO;

    uint8_t sense_key, asc, ascq;
    int rc = laser_test_unit_ready_locked(entry, &sense_key, &asc, &ascq);

    pthread_mutex_unlock(&entry->io_lock);

    if (rc == BOT_FAIL_NO_DEVICE) {
        entry->device_gone = 1;
        return LASER_ERR_NO_DEVICE;
    }
    if (rc == 0) {
        out->medium = LASER_MEDIUM_PRESENT;
        return LASER_OK;
    }

    switch (laser_medium_sense(sense_key, asc, ascq)) {
    case MEDIUM_SENSE_TRAY_OPEN:
        out->tray_open = 1;
        /* fall through */
    case MEDIUM_SENSE_ABSENT:
        out->medium = LASER_MEDIUM_ABSENT;
        return LASER_OK;
    case MEDIUM_SENSE_LOADING:
        out->medium = LASER_MEDIUM_PRESENT;
        return LASER_OK;
    case MEDIUM_SENSE_START_NEEDED:
    case MEDIUM_SENSE_INTERVENTION:
    case MEDIUM_SENSE_OTHER:
        break;
    }
    if (sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION &&
        asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
        ascq == SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED) {
        out->event = LASER_MEDIA_NEW;
        return LASER_OK;
    }
    return sense_key == 0xff ? LASER_ERR_IO : LASER_OK;
}

static laser_status_t media_poll_entry(laser_entry_t *entry, int token,
                                       laser_media_status_t *out)
{
    if (laser_is_cancelled(entry))
        return LASER_ERR_CANCELLED;
    if (entry->device_gone)
        return LASER_ERR_NO_DEVICE;

    laser_status_t st = LASER_ERR_REFUSED;
    if (!entry->media_events_unsupported) {
        st = media_events(entry, out);
        if (st == LASER_ERR_REFUSED) {
            LOGI("token=%d, usb %04x:%04x: no media events, following the "
                 "medium with TEST UNIT READY", token, entry->vid, entry->pid);
            entry->media_events_unsupported = 1;
            memset(out, 0, sizeof(*out));
        }
    }
    if (st == LASER_ERR_REFUSED)
        st = media_from_unit_ready(entry, out);

    /* Another disc, or none: the blocks known not to read were the last
     * one's, and its consumers are told (laser_media_generation()). */
    if (st == LASER_OK && (out->event == LASER_MEDIA_NEW ||
                           out->event == LASER_MEDIA_REMOVED))
        laser_forget_disc(entry, token);

    /* A new medium: the registration's wait answered for the one before, or
     * for an empty drive. Its "never became ready" would otherwise outlive
     * it for as long as a consumer keeps the device registered, and turn the
     * new disc away unprobed; lowered, the next laser_token_settle() waits
     * for this one instead. */
    if (st == LASER_OK && entry->wait_state != LASER_WAIT_READY &&
        (out->event == LASER_MEDIA_NEW ||
         out->medium == LASER_MEDIUM_PRESENT)) {
        pthread_mutex_lock(&entry->io_lock);
        entry->wait_state = LASER_WAIT_READY;
        pthread_mutex_unlock(&entry->io_lock);
        LOGI("token=%d: new medium, readiness to be checked again", token);
    }
    return st;
}

laser_status_t laser_media_poll(int token, laser_media_status_t *out)
{
    if (out == NULL) {
        LOGE("token=%d: laser_media_poll: NULL status", token);
        return LASER_ERR_INVALID;
    }
    memset(out, 0, sizeof(*out));

    /* Pinned: a poller often runs under a claim that is its application's,
     * released from another thread - on an unplug above all, which is when
     * a poll is most likely to be waiting on the drive. */
    laser_entry_t *entry = laser_pin(token);
    if (entry == NULL)
        return LASER_ERR_NO_SUCH_TOKEN;

    laser_status_t st = media_poll_entry(entry, token, out);
    laser_unpin(entry);
    return st;
}

laser_status_t laser_media_lock(int token, int locked)
{
    uint8_t cdb[6] = { 0 };
    cdb[0] = 0x1e;              /* PREVENT ALLOW MEDIUM REMOVAL */
    cdb[4] = locked ? 0x01 : 0x00;

    laser_status_t st = laser_scsi_cdb(token, cdb, sizeof(cdb), NULL, 0, 0,
                                       NULL);
    LOGI("token=%d: tray %s -> %d", token, locked ? "lock" : "unlock",
         (int)st);
    return st;
}

laser_status_t laser_capacity(int token, uint32_t *num_blocks)
{
    if (num_blocks == NULL) {
        LOGE("token=%d: laser_capacity() with no place for the answer",
             token);
        return LASER_ERR_INVALID;
    }

    uint8_t cdb[10] = { 0 };
    cdb[0] = 0x25;  /* READ CAPACITY(10) */

    uint8_t data[8] = { 0 };
    int actual = 0;
    laser_status_t st = laser_scsi_cdb(token, cdb, sizeof(cdb), data,
                                       sizeof(data), 1, &actual);
    if (st != LASER_OK) {
        LOGW("token=%d: READ CAPACITY failed (status %d)", token, (int)st);
        return st;
    }
    if (actual < (int)sizeof(data)) {
        LOGW("token=%d: READ CAPACITY answered %d of %d bytes", token, actual,
             (int)sizeof(data));
        return LASER_ERR_IO;
    }

    const uint32_t last_lba = ((uint32_t)data[0] << 24) |
                              ((uint32_t)data[1] << 16) |
                              ((uint32_t)data[2] << 8) | (uint32_t)data[3];
    const uint32_t block_size = ((uint32_t)data[4] << 24) |
                                ((uint32_t)data[5] << 16) |
                                ((uint32_t)data[6] << 8) | (uint32_t)data[7];

    /* READ CAPACITY(10)'s overflow sentinel: implausible on optical media. */
    if (last_lba == 0xFFFFFFFFu) {
        LOGW("token=%d: READ CAPACITY reports the 32-bit overflow sentinel",
             token);
        return LASER_ERR_IO;
    }
    if (block_size != 2048) {
        LOGW("token=%d: READ CAPACITY reports %u-byte blocks, expected 2048",
             token, block_size);
        return LASER_ERR_IO;
    }

    *num_blocks = last_lba + 1;
    return LASER_OK;
}

laser_status_t laser_media_eject(int token)
{
    uint8_t cdb[6] = { 0 };
    cdb[0] = 0x1b;  /* START STOP UNIT */
    cdb[1] = 0x01;  /* IMMED: the tray takes seconds */
    cdb[4] = 0x02;  /* LOEJ, START clear: eject */

    laser_status_t st = laser_scsi_cdb(token, cdb, sizeof(cdb), NULL, 0, 0,
                                       NULL);
    LOGI("token=%d: eject -> %d", token, (int)st);
    return st;
}
