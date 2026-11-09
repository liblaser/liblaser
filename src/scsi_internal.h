/*****************************************************************************
 * scsi_internal.h: what the files of the SCSI layer share
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
 * The vocabulary and the helpers that scsi.c, ready.c, read.c and medium.c
 * share, and nothing outside them includes: sense codes, what a sense says of
 * the medium, GET EVENT STATUS NOTIFICATION's media class, and the commands
 * below the retries of laser_scsi_cdb().
 *****************************************************************************/

#ifndef LASER_SCSI_INTERNAL_H
#define LASER_SCSI_INTERNAL_H

#include <stdint.h>

#include "laser_internal.h"

/* Fixed-format sense data (SPC): sense key in the low nibble of byte 2,
 * Additional Sense Code (ASC) in byte 12, its qualifier (ASCQ) in byte 13.
 * Values checked against T10's assignment list (t10.org/lists/asc-num.txt).
 *
 * Whether the ASCQ must be matched depends on the ASC: every 3Ah qualifier
 * means "no medium" to an ordinary command, while under 28h only 00h means a
 * changed medium - 02h is a dual-layer DVD crossing layers mid-playback. */
#define SCSI_SENSE_KEY_NOT_READY        0x02
#define SCSI_SENSE_KEY_UNIT_ATTENTION   0x06

/* Sense keys that no retry can turn into a success: the drive understood
 * the command and refuses it as issued - a malformed CDB, an LBA past the
 * end, a scrambled sector before authentication. */
#define SCSI_SENSE_KEY_ILLEGAL_REQUEST  0x05
#define SCSI_SENSE_KEY_DATA_PROTECT     0x07

/* A sector the drive could not read, its own retries and error correction
 * spent: see LASER_READ_MAX_MEDIUM_ERRORS. */
#define SCSI_SENSE_KEY_MEDIUM_ERROR     0x03

/* A read the drive answered with a medium error is tried twice, not
 * LASER_MAX_RETRIES times: the drive has already retried the sector itself,
 * and a disc it reads poorly - read fine by another drive - fails one
 * attempt in several. A JMicron bridge stalls three seconds on each, and
 * the full budget made a few such sectors outlast classification's minute.
 * read_chunked() tries the smaller reads it narrows such a failure down
 * with once each. */
#define LASER_READ_MAX_MEDIUM_ERRORS 2

/* Under ILLEGAL REQUEST, read after a refused START STOP UNIT: "no such
 * opcode" and "not in this shape" call for opposite next steps. */
#define SCSI_ASC_INVALID_OPCODE         0x20
#define SCSI_ASC_INVALID_CDB_FIELD      0x24

/* 6Fh COPY PROTECTION KEY EXCHANGE FAILURE and neighbours, all under ILLEGAL
 * REQUEST, with three different remedies:
 *
 *   6Fh/00h AUTHENTICATION FAILURE
 *   6Fh/01h KEY NOT PRESENT
 *   6Fh/02h KEY NOT ESTABLISHED   -> re-authenticate; the sector is not
 *                                    permanently unreadable.
 *   6Fh/03h READ OF SCRAMBLED SECTOR WITHOUT AUTHENTICATION
 *                                 -> unreadable as things stand.
 *   6Fh/04h MEDIA REGION CODE IS MISMATCHED TO LOGICAL UNIT REGION
 *   6Fh/05h DRIVE REGION MUST BE PERMANENT/REGION RESET COUNT ERROR
 *                                 -> not an authentication problem; the user
 *                                    must be told, and the drive's region
 *                                    never changed on their behalf. */
#define SCSI_ASC_COPY_PROTECTION        0x6f
#define SCSI_ASCQ_CP_AUTH_FAILURE       0x00
#define SCSI_ASCQ_CP_KEY_NOT_PRESENT    0x01
#define SCSI_ASCQ_CP_KEY_NOT_ESTABLISHED 0x02
#define SCSI_ASCQ_CP_SCRAMBLED          0x03
#define SCSI_ASCQ_CP_REGION_MISMATCH    0x04
#define SCSI_ASCQ_CP_REGION_PERMANENT   0x05

/* 04h: not ready, 04h/01h being "becoming ready". In the spin-up wait it
 * raises the ceiling and is recorded: a drive that leaves the bus right after
 * saying it was drawing spindle current. In laser_scsi_cdb() it is waited
 * out rather than retried. */
#define SCSI_ASC_BECOMING_READY         0x04

#define SCSI_ASC_MEDIUM_NOT_PRESENT     0x3a
/* The qualifiers under 3Ah, which the spin-up wait tells apart. 3Ah/02h, tray
 * OPEN, is a physical fact no command or wait changes. 3Ah/00h and 3Ah/01h
 * only say the drive has no medium state, which is also what a parked drive
 * answers about a disc in its own closed tray. */
#define SCSI_ASCQ_MEDIUM_NOT_PRESENT      0x00
#define SCSI_ASCQ_TRAY_CLOSED             0x01
#define SCSI_ASCQ_TRAY_OPEN               0x02

#define SCSI_ASC_MEDIUM_MAY_HAVE_CHANGED 0x28
/* 28h/00h NOT READY TO READY CHANGE, MEDIUM MAY HAVE CHANGED - the only
 * qualifier under ASC 28h that actually means the disc was swapped. */
#define SCSI_ASCQ_MEDIUM_MAY_HAVE_CHANGED 0x00
/* 28h/02h FORMAT-LAYER MAY HAVE CHANGED - dual-layer boundary crossing,
 * transient and retryable; explicitly NOT an ejection. */
#define SCSI_ASCQ_FORMAT_LAYER_CHANGED    0x02

/* The ceiling once the drive has answered 04h/01h BECOMING READY: unlike the
 * other "not ready" answers, that one says the drive is working on it. A
 * Blu-ray drive behind a 152d:0583 bridge was seen answering it for more than
 * 15 s while loading a disc just inserted. Also the ceiling of a command
 * answered it, counted from that answer. */
#define LASER_BECOMING_READY_MAX_WALL_MS 30000

/* What a sense says of the medium. */
typedef enum {
    MEDIUM_SENSE_OTHER = 0,
    /** 02h/3Ah, tray closed or no qualifier: an empty tray, and a parked
     * drive holding a disc alike. */
    MEDIUM_SENSE_ABSENT,
    /** 02h/3Ah/02h. */
    MEDIUM_SENSE_TRAY_OPEN,
    /** 02h/04h, becoming ready: the drive is loading or spinning up. */
    MEDIUM_SENSE_LOADING,
} medium_sense_t;

medium_sense_t laser_medium_sense(uint8_t sense_key, uint8_t asc,
                                  uint8_t ascq);

/* GET EVENT STATUS NOTIFICATION (opcode 4Ah), media class: the event codes
 * in the low nibble of the descriptor's first byte, the status bits in its
 * second. */
#define GESN_CLASS_MEDIA            4
#define GESN_MEDIA_EJECT_REQUEST    1
#define GESN_MEDIA_NEW              2
#define GESN_MEDIA_REMOVAL          3
#define GESN_MEDIA_CHANGED          4
#define GESN_MEDIA_STATUS_TRAY_OPEN 0x01
#define GESN_MEDIA_STATUS_PRESENT   0x02

/* An answer of GET EVENT STATUS NOTIFICATION, media class. */
typedef struct {
    int     nea;        /* no event of the class asked for is supported */
    int     cls;        /* the class answered */
    int     event;      /* GESN_MEDIA_* */
    int     tray_open;
    int     present;    /* not trusted: see laser_medium_t */
    uint8_t raw[4];     /* bytes 2 to 5, for the log */
    int     length;     /* bytes received */
} gesn_media_t;

/* scsi.c */

/* Why a command failed, as far as the drive said: for a caller that reacts
 * to one answer differently from the others. */
typedef struct {
    /* The LASER_ERR_IO returned is the drive answering MEDIUM ERROR: it
     * reached the sector and could not read it - not a drive that did not
     * answer, nor a transport failure. */
    int     medium_error;
    /* The last sense REQUEST SENSE read, 0xff/00/00 if none was. */
    uint8_t sense_key;
    uint8_t asc;
    uint8_t ascq;
} laser_failure_t;

/* laser_scsi_cdb(), giving up on a read after @p max_medium_errors attempts
 * the drive answered with a medium error rather than
 * LASER_READ_MAX_MEDIUM_ERRORS - 1 for once - and saying why it failed
 * through @p why, if not NULL. */
laser_status_t laser_scsi_cdb_read(int token,
                                   const uint8_t *cdb, int cdb_len,
                                   uint8_t *data, int data_len,
                                   int data_in, int *actual_len,
                                   int max_medium_errors,
                                   laser_failure_t *why);

/* One command, without the retries of laser_scsi_cdb(), and why it failed:
 * when its CSW says FAIL, the sense REQUEST SENSE reads, into @p sense_key,
 * @p asc and @p ascq - left 0xff/00/00 otherwise, and not asked for when
 * @p sense_key is NULL. A data phase, if any, is DATA-IN.
 *
 * Returns laser_bot_send_locked()'s code; @p actual_len and @p csw_status
 * may be NULL.
 *
 * Caller MUST already hold entry->io_lock. */
int laser_command_locked(laser_entry_t *entry, const uint8_t *cdb,
                         int cdb_len, uint8_t *data, int data_len,
                         int *actual_len, int *csw_status,
                         uint8_t *sense_key, uint8_t *asc, uint8_t *ascq);

/* SCSI TEST UNIT READY (opcode 0x00): no data phase, the CSW status alone
 * says whether the unit is ready, the sense why not - as
 * laser_command_locked().
 *
 * Returns laser_bot_send_locked()'s code as is: 0 means ready,
 * BOT_FAIL_NO_DEVICE means gone, anything else not ready yet.
 *
 * Caller MUST already hold entry->io_lock. */
int laser_test_unit_ready_locked(laser_entry_t *entry, uint8_t *sense_key,
                                 uint8_t *asc, uint8_t *ascq);

/* Sleeps @p ms with io_lock released, then takes it again: for the delays
 * between the attempts of a command, and the rounds of a wait, so that the
 * other consumers of the drive - a poll, the player, libdvdcss - are not held
 * up for the whole of a retry budget, or of a thirty-second wait for a drive
 * becoming ready. Whatever the caller knew of the drive's state may have
 * changed by the time it returns.
 *
 * Caller MUST hold entry->io_lock; holds it again on return. */
void laser_sleep_unlocked(laser_entry_t *entry, int ms);

/* Now on CLOCK_MONOTONIC, in milliseconds: for the budgets, which the
 * system clock being stepped - as Android does routinely after a boot or a
 * network change - must not stretch or cut, and for a time kept in an
 * entry. */
int64_t laser_monotonic_ms_now(void);

/* The delay before another attempt. An idempotent command lets the other
 * consumers in meanwhile (laser_sleep_unlocked()); a command changing CSS
 * state keeps the drive, its sequence being the one thing io_lock must not
 * let another command into. */
void laser_retry_delay(laser_entry_t *entry, int idempotent);

/* medium.c */

/* One GET EVENT STATUS NOTIFICATION, polled, media class, decoded into
 * @p out. Returns laser_bot_send_locked()'s code, -1 for an answer too short
 * to decode; the sense as laser_command_locked(), @p sense_key may be NULL.
 *
 * Caller MUST already hold entry->io_lock. */
int laser_gesn_media_locked(laser_entry_t *entry, gesn_media_t *out,
                            uint8_t *sense_key, uint8_t *asc, uint8_t *ascq);

/* read.c */

/* The disc changed, or left: what was known of its blocks goes with it.
 * Takes entry->io_lock: the caller MUST NOT hold it. */
void laser_forget_disc(laser_entry_t *entry, int token);

#endif /* LASER_SCSI_INTERNAL_H */
