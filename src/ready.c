/*****************************************************************************
 * ready.c: bringing a drive up
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
 * What registration runs once on a new drive, and laser_token_settle() again
 * on a registered one: the INQUIRY probe that finds its optical unit, the
 * tray unlock, and the wait for its medium to become ready - spin-up,
 * UNIT ATTENTIONs and a disc still loading included. Bounded in real time:
 * at registration it runs under the registry lock.
 *
 * Commands go through laser_command_locked() (scsi.c), or straight to
 * laser_bot_send_locked() for the INQUIRY probe, without the retries of
 * laser_scsi_cdb(): the wait is its own retry policy.
 *****************************************************************************/

#include <string.h>
#include <unistd.h>

#include "laser.h"
#include "laser_internal.h"
#include "scsi_internal.h"

/* Delay between two polls of the spin-up wait. */
#define LASER_SPINUP_DELAY_MS      500

/* Ceiling on the real time the spin-up wait may take. Real time rather than a
 * number of polls: a drive that stops answering without leaving the bus makes
 * each poll cost two full sets of phase timeouts, and a poll count would then
 * stretch into minutes - spent under the registry lock, blocking every
 * teardown and registration. */
#define LASER_SPINUP_MAX_WALL_MS   15000

/* UNIT ATTENTIONs retried at once, past the ceilings if need be: see the
 * wait loop. Bounded against a device that would answer nothing else. */
#define LASER_SPINUP_MAX_UNIT_ATTENTIONS 4

/* A shorter ceiling, for a drive answering 3Ah with a tray-closed or generic
 * qualifier, counted from the START STOP UNIT. Unlike "becoming ready", that
 * answer promises nothing - an empty tray gives it too - so it gets long
 * enough for a drive told to load to find its medium, and short enough that
 * an empty drive costs a pause rather than fifteen seconds. Whichever ceiling
 * comes first ends the wait. */
#define LASER_NO_MEDIUM_MAX_WALL_MS 4000

/* Once the drive has said it is becoming ready, a 3Ah answer no longer ends
 * the wait at LASER_NO_MEDIUM_MAX_WALL_MS: the drive is loading a disc, and
 * a 152d:0583 bridge's drive was seen answering 04h/01h for a second, then
 * 3Ah/01h, while still loading one just inserted. Polled this long after its
 * last "becoming ready", within LASER_BECOMING_READY_MAX_WALL_MS. */
#define LASER_NO_MEDIUM_AFTER_LOADING_MS 10000

/* Highest LUN probed, whatever the device claims: each unit costs an INQUIRY
 * at registration, and combo enclosures use two at most. */
#define LASER_MAX_LUN_PROBED 3

/* SCSI INQUIRY (SPC), byte 0 low 5 bits: peripheral device type. */
#define SCSI_PDT_MASK           0x1f
#define SCSI_PDT_DIRECT_ACCESS  0x00
#define SCSI_PDT_CD_DVD         0x05

/* INQUIRY allocation length: the 36-byte standard short form every device
 * must answer. */
#define SCSI_INQUIRY_ALLOC_LEN  36

/* INQUIRY attempts on LUN 0, and the first delay between them, which doubles
 * each time: 100, 200, ... 3200 ms, about six seconds over seven attempts.
 *
 * On a kernel that mounts optical media, a drive classified just after it was
 * plugged in is caught mid-command by usb-storage, whose work nobody will now
 * collect; the device needs time, not another command. A healthy one answers
 * the first attempt and sleeps for none of it. */
#define LASER_INQUIRY_MAX_ATTEMPTS 7
#define LASER_INQUIRY_RETRY_MS     100

/* One START STOP UNIT in the exact shape asked for. Returns the CSW status,
 * and fills the sense triple when that status is FAIL - the sense being what
 * tells a missing opcode from a wrong shape from a refusal of the drive's
 * own. */
static int start_stop_unit_locked(laser_entry_t *entry,
                                  uint8_t immed, uint8_t byte4,
                                  uint8_t *sense_key, uint8_t *asc,
                                  uint8_t *ascq)
{
    uint8_t cdb[6] = { 0 };
    cdb[0] = 0x1b;  /* START STOP UNIT */
    cdb[1] = immed; /* bit 0 IMMED */
    cdb[4] = byte4; /* bit 0 START, bit 1 LOEJ */

    int csw_status;
    int rc = laser_command_locked(entry, cdb, sizeof(cdb), NULL, 0, NULL,
                                  &csw_status, sense_key, asc, ascq);

    LOGI("token=%d: START STOP UNIT immed=%u byte4=0x%02x -> rc=%d csw=%d "
         "sense %02x/%02x/%02x",
         entry->token, immed, byte4, rc, csw_status,
         *sense_key, *asc, *ascq);

    return csw_status;
}

/* PREVENT ALLOW MEDIUM REMOVAL (opcode 1Eh) with Prevent = 00b: unlock the
 * tray. See laser_internal.h. */
void laser_allow_medium_removal(laser_entry_t *entry)
{
    uint8_t cdb[6] = { 0 };
    cdb[0] = 0x1e;  /* PREVENT ALLOW MEDIUM REMOVAL; byte 4 = 0: allow */

    pthread_mutex_lock(&entry->io_lock);

    int csw_status;
    uint8_t sense_key, asc, ascq;
    int rc = laser_command_locked(entry, cdb, sizeof(cdb), NULL, 0, NULL,
                                  &csw_status, &sense_key, &asc, &ascq);
    /* 06h UNIT ATTENTION reports an event - the drive's power-on reset, as
     * the first command it gets - not a refusal, and is cleared by being
     * reported: the same request once more (see spin_up_locked()). */
    if (rc != 0 && sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION) {
        LOGI("token=%d: PREVENT ALLOW MEDIUM REMOVAL met a UNIT ATTENTION "
             "(%02x/%02x), sending it again", entry->token, asc, ascq);
        rc = laser_command_locked(entry, cdb, sizeof(cdb), NULL, 0, NULL,
                                  &csw_status, &sense_key, &asc, &ascq);
    }

    pthread_mutex_unlock(&entry->io_lock);

    /* Not fatal either way: a drive that refuses still plays, its tray is
     * just left as the kernel left it. */
    if (rc == 0) {
        LOGI("token=%d: tray unlocked (PREVENT ALLOW MEDIUM REMOVAL, allow)",
             entry->token);
    } else {
        LOGW("token=%d: PREVENT ALLOW MEDIUM REMOVAL refused (rc=%d csw=%d "
             "sense %02x/%02x/%02x) - the tray may stay locked until the "
             "drive is unplugged",
             entry->token, rc, csw_status, sense_key, asc, ascq);
    }
}

/* Tell the drive to load and spin its medium up, escalating through the
 * shapes a bridge might accept.
 *
 * TEST UNIT READY is a status query: some firmware starts the mechanism on
 * it, some does not, and a drive of the latter kind would answer MEDIUM NOT
 * PRESENT about a readable disc forever. On Linux, sr_mod sends this command
 * when the device is opened; with the kernel driver detached, nothing else
 * would.
 *
 * Byte 4: bit 0 START, bit 1 LOEJ ("load first"). IMMED is set on the first
 * attempt because a blocking spin-up can outlast bot.c's fixed phase
 * timeouts.
 *
 *   1.  IMMED, START: the correct request, and the cheapest.
 *   1b. 06h UNIT ATTENTION: the same request once more. It reports an event
 *       - a reset, a medium change - not a refusal, and is cleared by being
 *       reported: a 152d:0583 bridge answers 06h/29h to the first command
 *       after power-on. Its second answer goes down the ladder below.
 *   2a. 05h/20h INVALID OPCODE: stop. Every other shape is the same opcode.
 *   2b. 02h/04h NOT READY, BECOMING READY: stop. The drive is already
 *       loading, and the caller's wait covers that.
 *   2c. 05h/24h INVALID FIELD IN CDB: retry without IMMED, the field ATAPI
 *       bridges most often lack. This one may block for the whole spin-up,
 *       and is given the time: see SLOW_CSW_PHASE_TIMEOUT_MS in bot.c.
 *   2d. anything else: retry with LOEJ, "load the medium" being a different
 *       request from "spin what you have". A 05h/24h answer to that names
 *       LOEJ, so the ladder ends there: a drive with no motorised load
 *       refuses it with or without IMMED, as a 152d:0583 bridge showed.
 *
 * Never with the tray open: LOEJ would close it. Best-effort; the caller
 * polls either way.
 *
 * Caller MUST already hold entry->io_lock. */
static void spin_up_locked(laser_entry_t *entry)
{
    uint8_t sense_key, asc, ascq;

    int status = start_stop_unit_locked(entry, 1, 0x01,
                                        &sense_key, &asc, &ascq);
    if (status != USB_BOT_STATUS_PASS &&
        sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION) {
        LOGI("token=%d: START STOP UNIT met a UNIT ATTENTION (%02x/%02x), "
             "sending it again", entry->token, asc, ascq);
        status = start_stop_unit_locked(entry, 1, 0x01,
                                        &sense_key, &asc, &ascq);
    }
    if (status == USB_BOT_STATUS_PASS) {
        return;
    }

    if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST &&
        asc == SCSI_ASC_INVALID_OPCODE) {
        LOGW("token=%d: drive does not implement START STOP UNIT - nothing "
             "here can ask it to load a medium", entry->token);
        return;
    }

    if (laser_medium_sense(sense_key, asc, ascq) == MEDIUM_SENSE_LOADING) {
        LOGI("token=%d: drive already becoming ready, no further START STOP "
             "UNIT", entry->token);
        return;
    }

    if (sense_key == SCSI_SENSE_KEY_ILLEGAL_REQUEST &&
        asc == SCSI_ASC_INVALID_CDB_FIELD) {
        LOGI("token=%d: START STOP UNIT refused the CDB, retrying without "
             "IMMED (this one may block for the whole spin-up)",
             entry->token);
        start_stop_unit_locked(entry, 0, 0x01, &sense_key, &asc, &ascq);
        return;
    }

    /* The tray is open: LOEJ would close it on the user. The wait that
     * follows sees the open tray and ends at once. */
    if (laser_medium_sense(sense_key, asc, ascq) == MEDIUM_SENSE_TRAY_OPEN) {
        LOGI("token=%d: tray open, not loading it", entry->token);
        return;
    }

    LOGI("token=%d: START STOP UNIT refused for its own reasons, retrying "
         "with LOEJ set", entry->token);
    start_stop_unit_locked(entry, 1, 0x03, &sense_key, &asc, &ascq);
}

/* What the drive believes about its tray and medium, for the log of a
 * given-up wait.
 *
 * Diagnostic only, and not ground truth: it is the same firmware's belief as
 * the sense data, and has reported "tray closed, medium absent" for a disc
 * read moments later. Decisions are made on the sense qualifier.
 *
 * Caller MUST already hold entry->io_lock. */
static void log_media_status_locked(laser_entry_t *entry)
{
    gesn_media_t g;
    int rc = laser_gesn_media_locked(entry, &g, NULL, NULL, NULL);
    if (rc != 0) {
        LOGI("token=%d: media status unavailable (rc=%d, %d bytes) - the "
             "drive does not support 4Ah, or would not answer it",
             entry->token, rc, g.length);
        return;
    }

    static const char *const events[] = {
        "no change", "eject request", "new media", "media removal",
        "media changed",
    };

    LOGI("token=%d: media status: tray %s, medium %s, last event %s "
         "(nea=%d class=%d raw=%02x %02x %02x %02x)",
         entry->token,
         g.tray_open ? "OPEN" : "closed",
         g.present ? "PRESENT" : "absent",
         g.event < (int)(sizeof(events) / sizeof(events[0])) ? events[g.event]
                                                             : "unknown",
         g.nea, g.cls, g.raw[0], g.raw[1], g.raw[2], g.raw[3]);
}

/* Wake the drive and wait for its medium to become ready - see the contract
 * in laser_internal.h.
 *
 * A drive that has spun down, or never spun up since the disc went in,
 * answers the first data command NOT READY, and some firmware fails a READ
 * that arrives cold rather than starting the spin-up. So: one START STOP
 * UNIT, then TEST UNIT READY in a loop until the unit answers ready.
 *
 * The loop ends early on a tray open (3Ah/02h), which no command closes, and
 * on a device that has left the bus. Other 3Ah answers keep it polling under
 * LASER_NO_MEDIUM_MAX_WALL_MS; everything else under LASER_SPINUP_MAX_WALL_MS,
 * raised to LASER_BECOMING_READY_MAX_WALL_MS once the drive has said it is
 * becoming ready.
 *
 * Not cancellable. At registration it runs inside laser_acquire(), under
 * g_registry_lock, on an entry not yet published; laser_token_settle() may
 * run it again on a published one. */
void laser_wait_until_ready(laser_entry_t *entry)
{
    const int64_t started = laser_monotonic_ms_now();

    pthread_mutex_lock(&entry->io_lock);

    /* Asked before the first poll rather than on a first 3Ah answer: a drive
     * answering "becoming ready" as a generic busy state would otherwise
     * never be told to load. One command too many on a drive that did not
     * need it. */
    spin_up_locked(entry);

    /* When START STOP UNIT went out: the start of the no-medium ceiling,
     * since "not yet" only means something once the drive was asked to
     * load. */
    const int64_t no_medium_since = laser_monotonic_ms_now();

    /* Not ready while the wait runs: only the ready return below says
     * otherwise. Set again on every way out, since a poll may change it
     * while the wait sleeps. */
    entry->wait_state = LASER_WAIT_NOT_READY;

    /* Whether the drive said it was spinning up, for the device-gone report
     * below and the no-medium ceiling, and when it last said so. */
    int becoming_ready = 0;
    int64_t becoming_ready_last = started;
    int unit_attentions = 0;

    /* Whether this wait has already said the medium left: once is enough
     * for laser_entry_t::media_generation, however many polls say it. */
    int medium_left = 0;

    /* Bounded by the ceilings alone: every path below either returns, sleeps
     * and meets a ceiling check, or is a UNIT ATTENTION retry, itself
     * bounded. */
    for (int attempt = 1; ; attempt++) {
        uint8_t sense_key, asc, ascq;
        int rc = laser_test_unit_ready_locked(entry, &sense_key, &asc, &ascq);
        const int64_t now = laser_monotonic_ms_now();

        if (rc == 0) {
            LOGI("token=%d: unit ready (attempt %d, %lldms)",
                 entry->token, attempt, (long long)(now - started));
            entry->wait_state = LASER_WAIT_READY;
            pthread_mutex_unlock(&entry->io_lock);
            return;
        }

        if (rc == BOT_FAIL_NO_DEVICE) {
            LOGW("token=%d: device gone during spin-up wait, abandoning it",
                 entry->token);

            /* Leaving the bus while spinning up - when a cold drive draws its
             * peak current - is the signature of a port that cannot power it.
             * Every layer above only sees a device that disappeared. */
            if (becoming_ready) {
                LOGW("token=%d: it was spinning up when it went (%lldms in) - "
                     "suspect the port's power budget, not the drive: try a "
                     "powered hub or a Y-cable",
                     entry->token, (long long)(now - started));
            }

            entry->wait_state = LASER_WAIT_NOT_READY;
            pthread_mutex_unlock(&entry->io_lock);
            return;
        }

        /* Logged raw: 02h/3Ah, 02h/04h/01h and 06h/28h/00h call for different
         * handling, and the messages below do not show which arrived. */
        LOGI("token=%d: sense %02x/%02x/%02x (attempt %d)",
             entry->token, sense_key, asc, ascq, attempt);

        const medium_sense_t medium = laser_medium_sense(sense_key, asc, ascq);
        if (medium == MEDIUM_SENSE_LOADING) {
            becoming_ready = 1;
            becoming_ready_last = now;
        }

        /* The medium left, or another came: reported here to this wait and
         * to nobody else, so said here to every consumer of the token. At
         * registration there is none yet, and nothing to forget. */
        if ((medium == MEDIUM_SENSE_ABSENT ||
             medium == MEDIUM_SENSE_TRAY_OPEN) && !medium_left) {
            medium_left = 1;
            laser_disc_changed_locked(entry);
        }
        if (sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION &&
            asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
            ascq == SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED) {
            medium_left = 0;
            laser_disc_changed_locked(entry);
        }

        if (medium == MEDIUM_SENSE_TRAY_OPEN) {
            LOGW("token=%d: tray open (attempt %d), giving up wait",
                 entry->token, attempt);
            log_media_status_locked(entry);
            entry->wait_state = LASER_WAIT_TRAY_OPEN;
            pthread_mutex_unlock(&entry->io_lock);
            return;
        }

        if (medium == MEDIUM_SENSE_ABSENT) {
            /* An empty tray and a parked drive holding a disc alike. Keep
             * polling; the shorter ceiling bounds the empty case - unless the
             * drive was loading a disc, see LASER_NO_MEDIUM_AFTER_LOADING_MS. */
            const int64_t no_medium_ms = now - no_medium_since;
            const int loading = becoming_ready &&
                now - becoming_ready_last < LASER_NO_MEDIUM_AFTER_LOADING_MS &&
                now - started < LASER_BECOMING_READY_MAX_WALL_MS;
            if (no_medium_ms >= LASER_NO_MEDIUM_MAX_WALL_MS && !loading) {
                LOGW("token=%d: still no medium %lldms after START STOP UNIT "
                     "(attempt %d), giving up wait",
                     entry->token, (long long)no_medium_ms, attempt);
                log_media_status_locked(entry);
                entry->wait_state = LASER_WAIT_NO_MEDIUM;
                pthread_mutex_unlock(&entry->io_lock);
                return;
            }

            if (loading && no_medium_ms >= LASER_NO_MEDIUM_MAX_WALL_MS)
                LOGI("token=%d: no medium yet, waiting %dms (attempt %d, "
                     "%lldms since the drive said it was loading, up to %dms)",
                     entry->token, LASER_SPINUP_DELAY_MS, attempt,
                     (long long)(now - becoming_ready_last),
                     LASER_NO_MEDIUM_AFTER_LOADING_MS);
            else
                LOGI("token=%d: no medium yet, waiting %dms (attempt %d, "
                     "%lldms of %dms since START STOP UNIT)",
                     entry->token, LASER_SPINUP_DELAY_MS, attempt,
                     (long long)no_medium_ms, LASER_NO_MEDIUM_MAX_WALL_MS);
            laser_sleep_unlocked(entry, LASER_SPINUP_DELAY_MS);
            continue;
        }

        /* Cleared by being reported, and most often the drive saying it has
         * just loaded its medium: the next TEST UNIT READY is the likely
         * "ready", so it goes out at once - before the ceiling check, which
         * would otherwise give up on a drive that became ready on the last
         * round. */
        if (sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION &&
            ++unit_attentions <= LASER_SPINUP_MAX_UNIT_ATTENTIONS) {
            continue;
        }

        /* Checked before committing to another round, so that the ceiling
         * bounds the time actually spent; after the sense checks, so that an
         * open tray or a missing medium ends the wait on its own terms. */
        const int64_t elapsed = now - started;
        const int ceiling = becoming_ready ? LASER_BECOMING_READY_MAX_WALL_MS
                                           : LASER_SPINUP_MAX_WALL_MS;
        if (elapsed >= ceiling) {
            LOGW("token=%d: spin-up wall-clock budget exhausted (%lldms over "
                 "%d attempts), proceeding anyway",
                 entry->token, (long long)elapsed, attempt);
            break;
        }

        LOGI("token=%d: drive not ready, waiting %dms (attempt %d, %lldms "
             "of %dms elapsed)",
             entry->token, LASER_SPINUP_DELAY_MS, attempt, (long long)elapsed,
             ceiling);
        laser_sleep_unlocked(entry, LASER_SPINUP_DELAY_MS);
    }

    LOGW("token=%d: drive did not become ready after %lldms, proceeding "
         "anyway", entry->token,
         (long long)(laser_monotonic_ms_now() - started));
    entry->wait_state = LASER_WAIT_NOT_READY;
    pthread_mutex_unlock(&entry->io_lock);
}

/* See laser_internal.h. The sense of a failed TEST UNIT READY is read: a
 * medium gone or changed is reported once, here if this is the command that
 * meets it, and the other consumers of the token learn it from
 * laser_entry_t::media_generation alone.
 *
 * A drive whose wait already ran out ("never became ready") is checked once
 * more but not waited for again: ready now, its state is ready; still not
 * ready, it keeps it - a new medium resets it too (laser_media_poll()). */
void laser_token_settle(int token)
{
    laser_entry_t *entry = laser_lookup(token);
    if (entry == NULL || entry->device_gone || laser_is_cancelled(entry))
        return;

    pthread_mutex_lock(&entry->io_lock);
    const int was_not_ready = entry->wait_state != LASER_WAIT_READY;
    uint8_t sense_key = 0xff, asc = 0, ascq = 0;
    int rc = laser_test_unit_ready_locked(entry, &sense_key, &asc, &ascq);
    int loading = 0;

    if (rc != 0 && rc != BOT_FAIL_NO_DEVICE) {
        const medium_sense_t medium = laser_medium_sense(sense_key, asc, ascq);
        if (medium == MEDIUM_SENSE_ABSENT ||
            medium == MEDIUM_SENSE_TRAY_OPEN ||
            (sense_key == SCSI_SENSE_KEY_UNIT_ATTENTION &&
             asc == SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED &&
             ascq == SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED))
            laser_disc_changed_locked(entry);
    }

    if (rc == 0) {
        entry->wait_state = LASER_WAIT_READY;
    } else if (rc != BOT_FAIL_NO_DEVICE && was_not_ready) {
        /* Not waited for again, but the reason is brought up to date: the
         * tray may have been opened, or closed on an empty drive, since. */
        switch (laser_medium_sense(sense_key, asc, ascq)) {
        case MEDIUM_SENSE_TRAY_OPEN:
            entry->wait_state = LASER_WAIT_TRAY_OPEN;
            break;
        case MEDIUM_SENSE_ABSENT:
            entry->wait_state = LASER_WAIT_NO_MEDIUM;
            break;
        case MEDIUM_SENSE_LOADING:
            loading = 1;
            break;
        case MEDIUM_SENSE_OTHER:
            break;
        }
    }
    pthread_mutex_unlock(&entry->io_lock);

    if (rc == 0 && was_not_ready)
        LOGI("token=%d: drive ready now, after its wait ran out", token);
    if (rc == 0 || rc == BOT_FAIL_NO_DEVICE || (was_not_ready && !loading))
        return;
    if (loading)
        LOGI("token=%d: drive loading a disc now, after its wait ran out",
             token);

    LOGI("token=%d: registered drive not ready - disc changed or still "
         "loading, waiting for it", token);
    laser_wait_until_ready(entry);
}

/* One INQUIRY on the unit selected by entry->lun, under the probe timeouts:
 * at the read timeouts, an absent unit of a multi-LUN device would cost
 * eleven seconds of registration.
 *
 * @param pdt receives the peripheral device type on success.
 * @return 0 on success, BOT_FAIL_NO_DEVICE if the device is gone, -1 if the
 *         unit did not answer.
 */
static int inquiry_pdt(laser_entry_t *entry, int attempts, uint8_t *pdt)
{
    uint8_t cdb[6] = { 0x12, 0x00, 0x00, 0x00, SCSI_INQUIRY_ALLOC_LEN, 0x00 };
    uint8_t inq[SCSI_INQUIRY_ALLOC_LEN];

    for (int attempt = 1; attempt <= attempts; ++attempt) {
        int actual = 0;
        memset(inq, 0, sizeof(inq));

        pthread_mutex_lock(&entry->io_lock);
        entry->probe_timeouts = 1;
        int rc = laser_bot_send_locked(entry, cdb, sizeof(cdb),
                                       inq, sizeof(inq), 1, &actual, NULL);
        entry->probe_timeouts = 0;
        pthread_mutex_unlock(&entry->io_lock);

        if (rc == 0 && actual >= 1) {
            *pdt = inq[0] & SCSI_PDT_MASK;
            return 0;
        }

        if (rc == BOT_FAIL_NO_DEVICE) {
            LOGI("token=%d: device gone during INQUIRY", entry->token);
            return BOT_FAIL_NO_DEVICE;
        }

        if (attempt == attempts) {
            LOGI("token=%d: LUN %u INQUIRY unanswered (rc=%d, %d bytes, "
                 "attempt %d/%d, giving up)",
                 entry->token, entry->lun, rc, actual, attempt, attempts);
            break;
        }

        /* Doubling from LASER_INQUIRY_RETRY_MS: the wait is what the retry
         * consists of. */
        const int delay_ms = LASER_INQUIRY_RETRY_MS << (attempt - 1);

        LOGI("token=%d: LUN %u INQUIRY unanswered (rc=%d, %d bytes, "
             "attempt %d/%d), settling %dms",
             entry->token, entry->lun, rc, actual, attempt, attempts,
             delay_ms);

        usleep((useconds_t)delay_ms * 1000);
    }

    return -1;
}

int laser_probe_lun(laser_entry_t *entry)
{
    /* Set first, so that every early return leaves a usable value. */
    entry->lun = 0;

    int max_lun = laser_get_max_lun(entry);
    if (max_lun < 0) {
        /* Stalled or unanswered means a single unit, per the class
         * specification: the common case, not worth a warning. */
        LOGI("token=%d: GET MAX LUN unsupported or failed, assuming single LUN",
             entry->token);
        max_lun = 0;
    } else if (max_lun > 0) {
        LOGI("usb %04x:%04x: device reports %d logical units, looking for "
             "the optical one", entry->vid, entry->pid, max_lun + 1);
        if (max_lun > LASER_MAX_LUN_PROBED)
            max_lun = LASER_MAX_LUN_PROBED;
    }

    /* Ask each unit what it is, and take the first CD/DVD one. entry->lun
     * selects the unit the CBW addresses.
     *
     * Only LUN 0 is retried: its INQUIRY is the first command after the Mass
     * Storage Reset, which a bridge still settling may fail. Units 1..3 are
     * usually absent on the drives that report them, and retrying them would
     * multiply the cost of the case this keeps cheap. */
    uint8_t lun0_pdt = 0;
    int have_lun0_pdt = 0;

    for (int lun = 0; lun <= max_lun; lun++) {
        entry->lun = (uint8_t)lun;

        uint8_t pdt = 0;
        int rc = inquiry_pdt(entry, lun == 0 ? LASER_INQUIRY_MAX_ATTEMPTS : 1,
                             &pdt);

        if (rc == BOT_FAIL_NO_DEVICE) {
            entry->lun = 0;
            LOGW("token=%d: device left the bus during the LUN probe",
                 entry->token);
            return LASER_OPTICAL_GONE;
        }
        if (rc != 0)
            continue; /* Unit absent or unhappy; try the next one. */

        LOGI("token=%d: LUN %d peripheral device type 0x%02x",
             entry->token, lun, pdt);

        if (lun == 0) {
            lun0_pdt = pdt;
            have_lun0_pdt = 1;
        }

        if (pdt == SCSI_PDT_CD_DVD) {
            LOGI("token=%d: using LUN %d (optical)", entry->token, lun);
            return LASER_OPTICAL_YES; /* entry->lun already holds it. */
        }
    }

    /* No unit said optical: back to LUN 0 rather than wherever the loop
     * stopped. */
    entry->lun = 0;

    if (!have_lun0_pdt) {
        /* INQUIRY needs no medium: a device that answers it on no unit is not
         * talking at all. */
        LOGI("token=%d: INQUIRY unanswered on every unit", entry->token);
        return LASER_OPTICAL_NO_ANSWER;
    }

    if (max_lun > 0) {
        LOG_QUIRK(entry, "multi-LUN device but none identified as optical, "
                         "falling back to LUN 0");
    }

    /* Only a direct-access block device is rejected - a card reader, a USB
     * key, an external disk, which Android serves through its own storage
     * path. Any other unexpected type is accepted: losing a real drive to a
     * lying descriptor would be far worse than probing one device too many. */
    if (lun0_pdt == SCSI_PDT_DIRECT_ACCESS) {
        LOGI("token=%d: direct-access block device, not an optical drive",
             entry->token);
        return LASER_OPTICAL_NO;
    }

    return LASER_OPTICAL_YES;
}
