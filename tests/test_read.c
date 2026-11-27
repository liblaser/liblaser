/* read.c against a simulated command layer: laser_scsi_cdb_read() is
 * replaced by one that fails the reads touching a range of blocks, in the
 * way each case asks for, and counts the commands it is sent.
 *
 * What is tested is what a read makes of a failure: which blocks it
 * returns, which it records as unreadable, how many commands it spends, and
 * what it does to the transfer size. */

#include <assert.h>
#include <stdarg.h>

#include "../src/read.c"

#include "check.h"

static laser_entry_t entry;

/* How the simulated drive fails a read touching [bad_lo, bad_hi). */
static enum {
    BAD_NONE,       /* it does not */
    BAD_MEDIUM,     /* MEDIUM ERROR */
    BAD_NO_ANSWER,  /* not answered, the drive otherwise ready */
    BAD_OTHER,      /* an I/O error that says nothing of the blocks */
    BAD_LANDING,    /* MEDIUM ERROR, until a command far away has followed
                     * a failed one: a first landing the drive fails */
    BAD_LOST,       /* MEDIUM ERROR, and every read after it too - block 0
                     * included - until the disc is started again; then the
                     * blocks read if restart_cures */
    BAD_SPOTS,      /* MEDIUM ERROR on 4 blocks every 160: a weak zone */
} bad_mode;
static uint32_t bad_lo, bad_hi;

/* For BAD_LANDING: a landing has failed, and the drive has been far away
 * since. */
static int landing_failed, landed_away;

/* For BAD_LOST: the drive has lost its way; whether a restart lets it read
 * the blocks; whether it has been restarted. */
static int lost, restart_cures, restarted_once;

/* START STOP UNIT commands received, stops and starts. */
static int stops, starts;

/* A read not answered has sent the blocks before the bad one. */
static int sends_partial;

/* A bridge that fails every transfer above this many bytes; 0 for none. */
static int bridge_limit;

/* The next transfers above 32768 bytes to fail, in passing. */
static int transient_failures;

static int commands;

/* Of those, the reads failed on a spot of BAD_SPOTS; failed with BAD_MEDIUM. */
static int spot_failures;
static int medium_failures;

/* ---- what read.c calls ---- */

laser_entry_t *laser_pin(int token)
{
    (void) token;
    return &entry;
}

void laser_unpin(laser_entry_t *e)
{
    (void) e;
}

void laser_log(laser_log_level_t level, const char *fmt, ...)
{
    (void) level;
    (void) fmt;
}

/* A clock that does not move: a medium error is narrowed down for as long
 * as there is something to narrow. */
int64_t laser_monotonic_ms_now(void)
{
    return 1000;
}

/* Nobody stops a read here. */
int laser_must_stop(const laser_entry_t *e)
{
    (void) e;
    return 0;
}

/* No case here has the drive refuse a read. */
int laser_status_is_positional(int status)
{
    (void) status;
    return 0;
}

laser_status_t laser_scsi_cdb_read(int token, const uint8_t *cdb, int cdb_len,
                                   uint8_t *data, int data_len, int data_in,
                                   int *actual_len, int max_medium_errors,
                                   int max_resets, laser_failure_t *why)
{
    (void) token;
    (void) cdb_len;
    (void) data;
    (void) data_in;
    (void) max_medium_errors;
    (void) max_resets;

    commands++;

    /* Landing again asks for no explanation. */
    laser_failure_t ignored;
    if (why == NULL)
        why = &ignored;
    why->cause = LASER_FAILURE_OTHER;
    why->sense_key = 0xff;
    why->asc = why->ascq = 0;
    why->received = 0;

    if (bridge_limit != 0 && data_len > bridge_limit)
        return LASER_ERR_IO;
    if (transient_failures > 0 && data_len > 32768) {
        transient_failures--;
        return LASER_ERR_IO;
    }

    const uint32_t lba = (uint32_t)cdb[2] << 24 | (uint32_t)cdb[3] << 16 |
                         (uint32_t)cdb[4] << 8 | cdb[5];
    const uint32_t count = (uint32_t)cdb[7] << 8 | cdb[8];
    if (bad_mode == BAD_LOST) {
        const int in_range = lba < bad_hi && lba + count > bad_lo;
        if (lost || (in_range && !(restart_cures && restarted_once))) {
            lost = 1;
            why->cause = LASER_FAILURE_MEDIUM;
            return LASER_ERR_IO;
        }
    } else if (bad_mode == BAD_LANDING) {
        const uint32_t distance = lba >= bad_lo ? lba - bad_lo : bad_lo - lba;
        if (landing_failed && distance >= 65536)
            landed_away = 1;
        if (lba < bad_hi && lba + count > bad_lo && !landed_away) {
            landing_failed = 1;
            why->cause = LASER_FAILURE_MEDIUM;
            return LASER_ERR_IO;
        }
    } else if (bad_mode == BAD_SPOTS) {
        for (uint32_t spot = bad_lo; spot < bad_hi; spot += 160) {
            if (lba < spot + 4 && lba + count > spot) {
                spot_failures++;
                why->cause = LASER_FAILURE_MEDIUM;
                return LASER_ERR_IO;
            }
        }
    } else if (bad_mode != BAD_NONE && lba < bad_hi && lba + count > bad_lo) {
        if (bad_mode == BAD_MEDIUM) {
            medium_failures++;
            why->cause = LASER_FAILURE_MEDIUM;
        }
        if (bad_mode == BAD_NO_ANSWER)
            why->cause = LASER_FAILURE_NO_ANSWER;
        if (sends_partial && lba < bad_lo)
            why->received = (int)(bad_lo - lba) * 2048;
        return LASER_ERR_IO;
    }

    *actual_len = data_len;
    return LASER_OK;
}

/* START STOP UNIT, the one other command read.c sends. */
laser_status_t laser_scsi_cdb(int token, const uint8_t *cdb, int cdb_len,
                              uint8_t *data, int data_len, int data_in,
                              int *actual_len)
{
    (void) token;
    (void) cdb_len;
    (void) data;
    (void) data_len;
    (void) data_in;
    (void) actual_len;

    commands++;
    if (cdb[0] == 0x1b) {
        if (cdb[4] & 0x01) {
            starts++;
            lost = 0;
            restarted_once = 1;
        } else {
            stops++;
        }
    }
    return LASER_OK;
}

/* ---- helpers ---- */

/* The ranges recorded as unreadable. */
static int recorded_ranges(void)
{
    int n = 0;
    for (unsigned i = 0; i < LASER_UNREADABLE_SLOTS; i++)
        n += entry.unreadable[i].count > 0;
    return n;
}

/* A drive that has read 64 KiB in one command, and a disc of which nothing
 * is known. */
static void fresh_disc(void)
{
    memset(entry.unreadable, 0, sizeof(entry.unreadable));
    memset(entry.refused, 0, sizeof(entry.refused));
    entry.unreadable_next = 0;
    entry.unreadable_total = 0;
    entry.largest_ok_bytes = 65536;
    entry.max_transfer_bytes = 65536;
    entry.cap_strike_bytes = 0;
    bad_mode = BAD_NONE;
    sends_partial = 0;
    bridge_limit = 0;
    transient_failures = 0;
    commands = 0;
    landing_failed = landed_away = 0;
    lost = restart_cures = restarted_once = 0;
    stops = starts = 0;
    entry.restart_pending = 0;
    entry.restarts_in_area = 0;
    entry.band_start = entry.band_end = entry.band_skip = 0;
    /* No stream of reads under way: the next read lands. */
    entry.stream_end = 0;
}

static uint8_t buffer[256 * 2048];

/* ---- the cases ---- */

static void medium_error_is_narrowed_and_recorded(void)
{
    check_case("a medium error at blocks 100 to 103");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 100;
    bad_hi = 104;

    /* The blocks before it are returned, and it alone is recorded. */
    CHECK(laser_unreadable_total(1) == 0);
    CHECK(laser_read_blocks(1, 0, 128, buffer) == 100);
    CHECK(recorded_ranges() == 1);
    CHECK(laser_unreadable_blocks(1, 100, 128) == 4);
    CHECK(laser_unreadable_total(1) == 4);

    /* Asked for again, it is answered without the drive. */
    commands = 0;
    CHECK(laser_read_blocks(1, 100, 28, buffer) == LASER_ERR_IO);
    CHECK(commands == 0);
    /* And not counted again. */
    CHECK(laser_unreadable_total(1) == 4);
}

static void ranges_that_touch_are_merged(void)
{
    check_case("a bad region of 64 blocks, met 4 blocks at a time");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 200;
    bad_hi = 264;

    for (uint32_t lba = 200; lba < 264; ) {
        CHECK(laser_read_blocks(1, lba, 4, buffer) == LASER_ERR_IO);
        const int known = laser_unreadable_blocks(1, lba, 4096);
        CHECK(known > 0);
        if (known <= 0)
            break;
        lba += (uint32_t)known;
    }

    CHECK(recorded_ranges() == 1);
    CHECK(laser_unreadable_blocks(1, 200, 4096) == 64);
    /* Each block once, however the range grew. */
    CHECK(laser_unreadable_total(1) == 64);
}

static void unanswered_read_is_recorded(void)
{
    check_case("a read the drive does not answer, while ready");
    fresh_disc();
    bad_mode = BAD_NO_ANSWER;
    bad_lo = 40;
    bad_hi = 41;

    /* Landing: sent again once, then given up. */
    CHECK(laser_read_blocks(1, 32, 32, buffer) == LASER_ERR_IO);
    CHECK(commands == 2);
    CHECK(recorded_ranges() == 1);
    /* Not narrowed down: the whole transfer is given up, and counted. */
    CHECK(laser_unreadable_total(1) == 32);
}

static void read_through(uint32_t from, uint32_t to);

static void unanswered_in_a_stream_is_skipped(void)
{
    check_case("a read not answered in the middle of a stream");
    fresh_disc();
    bad_mode = BAD_NO_ANSWER;
    bad_lo = 5000;
    bad_hi = 5001;

    /* The stream reaching it: the chunk not answered, not sent again, and
     * 256 blocks from it skipped. */
    CHECK(laser_read_blocks(1, 4904, 96, buffer) == 96);
    commands = 0;
    CHECK(laser_read_blocks(1, 5000, 32, buffer) == LASER_ERR_IO);
    CHECK(commands == 1);
    CHECK(laser_unreadable_blocks(1, 5000, 1000) == 256);

    check_case("a read made ahead not answered in the middle of a stream");
    fresh_disc();
    bad_mode = BAD_NO_ANSWER;
    bad_lo = 5000;
    bad_hi = 5001;
    CHECK(laser_read_blocks(1, 4904, 96, buffer) == 96);

    /* Skipped by the read made ahead itself: the reader's read follows
     * without the drive. */
    commands = 0;
    CHECK(laser_read_blocks_ahead(1, 5000, 128, buffer) == LASER_ERR_IO);
    CHECK(commands == 1);
    CHECK(laser_read_blocks(1, 5000, 32, buffer) == LASER_ERR_IO);
    CHECK(commands == 1);
    CHECK(laser_unreadable_blocks(1, 5000, 1000) == 256);

    check_case("reads not answered all along a stream, skips doubling");
    fresh_disc();
    bad_mode = BAD_NO_ANSWER;
    bad_lo = 5000;
    bad_hi = 105000;
    CHECK(laser_read_blocks(1, 4904, 96, buffer) == 96);
    commands = 0;
    read_through(5000, 120000);
    /* 100000 blocks crossed in about ten failed reads, past them the
     * readable blocks' commands. */
    CHECK(commands < 12 + (120000 - 105000) / 32);
}

static void other_failure_records_nothing(void)
{
    check_case("an I/O failure that says nothing of the blocks");
    fresh_disc();
    bad_mode = BAD_OTHER;
    bad_lo = 40;
    bad_hi = 41;

    CHECK(laser_read_blocks(1, 32, 32, buffer) == LASER_ERR_IO);
    CHECK(recorded_ranges() == 0);

    /* So the blocks read once the drive is itself again. */
    bad_mode = BAD_NONE;
    CHECK(laser_read_blocks(1, 32, 32, buffer) == 32);
}

static void read_ahead_gives_up_at_once(void)
{
    check_case("a read-ahead reaching a bad region");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 100;
    bad_hi = 164;

    /* Three chunks of 32 read, one command on the fourth, nothing kept
     * against it. */
    CHECK(laser_read_blocks_ahead(1, 0, 128, buffer) == 96);
    CHECK(commands == 4);
    CHECK(recorded_ranges() == 0);

    commands = 0;
    CHECK(laser_read_blocks_ahead(1, 100, 128, buffer) == LASER_ERR_IO);
    CHECK(commands == 1);
    CHECK(recorded_ranges() == 0);
}

static void read_ahead_stops_before_known_blocks(void)
{
    check_case("a read-ahead towards blocks already recorded");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 100;
    bad_hi = 104;
    laser_read_blocks(1, 100, 4, buffer);

    CHECK(laser_read_blocks_ahead(1, 64, 128, buffer) == 36);
}

static void disc_change_forgets_everything(void)
{
    check_case("a change of disc");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 100;
    bad_hi = 104;
    laser_read_blocks(1, 100, 4, buffer);
    CHECK(recorded_ranges() == 1);

    CHECK(laser_unreadable_total(1) == 4);
    const unsigned generation = entry.media_generation;
    laser_forget_disc(&entry, 1);
    CHECK(recorded_ranges() == 0);
    CHECK(laser_unreadable_total(1) == 0);
    CHECK(entry.media_generation == generation + 1);
}

static void transfer_size_is_lowered_on_a_repeated_failure(void)
{
    check_case("a bridge that fails transfers above 20000 bytes");
    fresh_disc();
    entry.largest_ok_bytes = 0;
    bridge_limit = 20000;

    /* The first read gets through in smaller commands, and changes
     * nothing for the next. */
    CHECK(laser_read_blocks(1, 0, 32, buffer) == 32);
    CHECK(entry.max_transfer_bytes == 65536);

    /* The second finds the same limit: it is the device's. */
    CHECK(laser_read_blocks(1, 32, 32, buffer) == 32);
    CHECK(entry.max_transfer_bytes == 16384);

    /* From then on, no command is spent finding it. */
    commands = 0;
    CHECK(laser_read_blocks(1, 64, 32, buffer) == 32);
    CHECK(commands == 4);
}

static void transfer_size_survives_a_passing_failure(void)
{
    check_case("one large transfer failing in passing");
    fresh_disc();
    entry.largest_ok_bytes = 0;

    transient_failures = 1;
    CHECK(laser_read_blocks(1, 0, 32, buffer) == 32);
    CHECK(entry.max_transfer_bytes == 65536);

    /* The next large read succeeds: the first failure is forgotten. */
    commands = 0;
    CHECK(laser_read_blocks(1, 32, 32, buffer) == 32);
    CHECK(commands == 1);
    CHECK(entry.cap_strike_bytes == 0);

    /* So that another, later, is a first one again. */
    transient_failures = 1;
    laser_read_blocks(1, 64, 32, buffer);
    CHECK(entry.max_transfer_bytes == 65536);
}

static void blocks_sent_before_a_stall_are_returned(void)
{
    check_case("a read the drive stops answering at its 17th block");
    fresh_disc();
    bad_mode = BAD_NO_ANSWER;
    bad_lo = 16;
    bad_hi = 17;
    sends_partial = 1;

    /* The 16 blocks it sent are returned, the rest recorded - landing,
     * after sending it again once. */
    CHECK(laser_read_blocks(1, 0, 32, buffer) == 16);
    CHECK(commands == 2);

    commands = 0;
    CHECK(laser_read_blocks(1, 16, 16, buffer) == LASER_ERR_IO);
    CHECK(commands == 0);

    commands = 0;
    CHECK(laser_read_blocks(1, 0, 16, buffer) == 16);
    CHECK(commands == 1);

    /* A read-ahead returns them too, and records nothing. */
    fresh_disc();
    bad_mode = BAD_NO_ANSWER;
    bad_lo = 16;
    bad_hi = 17;
    sends_partial = 1;
    CHECK(laser_read_blocks_ahead(1, 0, 32, buffer) == 16);
    CHECK(commands == 1);
    CHECK(recorded_ranges() == 0);
}

static void failed_landing_is_read_after_landing_again(void)
{
    check_case("a first landing the drive fails");
    fresh_disc();
    bad_mode = BAD_LANDING;
    bad_lo = 21126816;
    bad_hi = 21126816 + 4096;

    /* The landing fails; away to block 0 and back, the chunk again: read.
     * Nothing recorded. */
    CHECK(laser_read_blocks(1, 21126816, 32, buffer) == 32);
    CHECK(commands == 3);
    CHECK(recorded_ranges() == 0);

    /* Near the start of the disc, away past the chunk rather than to 0. */
    fresh_disc();
    bad_mode = BAD_LANDING;
    bad_lo = 100;
    bad_hi = 200;
    CHECK(laser_read_blocks(1, 128, 32, buffer) == 32);
    CHECK(commands == 3);
    CHECK(recorded_ranges() == 0);
}

static void landing_again_is_once_a_read(void)
{
    check_case("an unreadable sector, landing again not curing it");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 3000;
    bad_hi = 3004;

    /* The failure, away and back, the chunk again; then the narrowing, 16,
     * 8 and 4 blocks - not away again. */
    CHECK(laser_read_blocks(1, 3000, 32, buffer) == LASER_ERR_IO);
    CHECK(commands == 1 + 1 + 1 + 3);
    CHECK(laser_unreadable_blocks(1, 3000, 32) == 4);

    /* Recorded: asked for again, answered without the drive. */
    commands = 0;
    CHECK(laser_read_blocks(1, 3000, 32, buffer) == LASER_ERR_IO);
    CHECK(commands == 0);
}

static void lost_drive_is_restarted(void)
{
    check_case("a drive lost after a failure, block 0 failing too");
    fresh_disc();
    bad_mode = BAD_LOST;
    restart_cures = 1;
    bad_lo = 10914751;
    bad_hi = 10916000;

    /* The failure; away to block 0, failed too; the disc stopped and
     * started; away again; the chunk again: read. Nothing recorded. */
    CHECK(laser_read_blocks(1, 10914784, 32, buffer) == 32);
    CHECK(stops == 1 && starts == 1);
    CHECK(commands == 6);
    CHECK(recorded_ranges() == 0);

    check_case("a drive lost, and failing after a restart too");
    fresh_disc();
    bad_mode = BAD_LOST;
    restart_cures = 0;
    bad_lo = 10914751;
    bad_hi = 10916000;

    /* The same, the chunk failing again: 2048 blocks recorded at once, not
     * narrowed down... */
    CHECK(laser_read_blocks(1, 10914784, 32, buffer) == LASER_ERR_IO);
    CHECK(stops == 1 && starts == 1);
    CHECK(commands == 6);
    CHECK(laser_unreadable_blocks(1, 10914784, 4096) == 2048);

    /* ...answered without the drive... */
    commands = 0;
    CHECK(laser_read_blocks(1, 10915000, 32, buffer) == LASER_ERR_IO);
    CHECK(commands == 0);

    /* ...and the disc restarted before the read past them: read. */
    CHECK(laser_read_blocks(1, 10914784 + 2048, 32, buffer) == 32);
    CHECK(stops == 2 && starts == 2);
    CHECK(commands == 3);

    check_case("a drive restarting does not help: two restarts, no more");
    fresh_disc();
    bad_mode = BAD_LOST;
    restart_cures = 0;
    bad_lo = 20000000;
    bad_hi = 20100000;
    for (uint32_t lba = 20000000; lba < 20020000; ) {
        const int r = laser_read_blocks(1, lba, 32, buffer);
        const int known = laser_unreadable_blocks(1, lba, 4096);
        if (r > 0) {
            lba += (uint32_t)r;
            continue;
        }
        CHECK(known > 0);
        if (known <= 0)
            break;
        lba += (uint32_t)known;
    }
    CHECK(starts == 2);

    check_case("a sector that does not read, block 0 reading: no restart");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 3000;
    bad_hi = 3004;
    laser_read_blocks(1, 3000, 32, buffer);
    CHECK(starts == 0 && stops == 0);
}

static void read_ahead_does_not_land_again(void)
{
    check_case("a read-ahead failing its landing");
    fresh_disc();
    bad_mode = BAD_LANDING;
    bad_lo = 3000;
    bad_hi = 3100;

    /* Tried once, given up: the reader's own read lands again. */
    CHECK(laser_read_blocks_ahead(1, 3010, 32, buffer) == LASER_ERR_IO);
    CHECK(commands == 1);
    CHECK(recorded_ranges() == 0);
}

/* Reads [from, to) as laser.c does: windows of 128 blocks, the blocks
 * known not to read taken as read. */
static void read_through(uint32_t from, uint32_t to)
{
    for (uint32_t lba = from; lba < to; ) {
        const int r = laser_read_blocks(1, lba, 128, buffer);
        if (r > 0) {
            lba += (uint32_t)r;
            continue;
        }
        const int known = laser_unreadable_blocks(1, lba, 128);
        CHECK(known > 0);
        if (known <= 0)
            break;
        lba += (uint32_t)known;
    }
}

static void damaged_stretch_is_skipped(void)
{
    check_case("a scratch 8000 blocks long, played through");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 10000;
    bad_hi = 18000;

    medium_failures = 0;
    read_through(9000, 30000);

    /* The first failure narrowed down, after landing again - five failed
     * reads - then skips doubling, one failed read each: about fifteen in
     * all, not thousands. */
    CHECK(medium_failures <= 20);
    /* Nothing before it is lost, and at most as much again after it. */
    CHECK(laser_unreadable_blocks(1, 9999, 1) == 0);
    CHECK(laser_unreadable_blocks(1, 10000, 20000) >= 8000);
    CHECK(laser_unreadable_blocks(1, 10000, 20000) <= 2 * 8000);
}

static void dead_band_is_crossed(void)
{
    check_case("1.4 million blocks that do not read, crossed");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 1000000;
    bad_hi = 2400000;
    medium_failures = 0;

    read_through(999000, 3000000);

    /* The first failure narrowed down, after landing again - five failed
     * reads - then a skip doubling at each failure: about twenty failed
     * reads, not tens of thousands. */
    CHECK(medium_failures <= 25);
    CHECK(laser_unreadable_blocks(1, 999999, 1) == 0);
    /* Read on past the band: at most as many blocks as it held lost after
     * it. */
    CHECK(laser_unreadable_blocks(1, 2400000 + 1400000, 1) == 0);
}

static void lone_bad_blocks_stay_narrow(void)
{
    check_case("two bad spots far apart");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 5000;
    bad_hi = 5004;

    read_through(4900, 5200);
    CHECK(laser_unreadable_blocks(1, 5000, 1000) == 4);

    /* Another spot, well past the first - out of its weak zone: narrowed
     * down too, not skipped. */
    bad_lo = 10000;
    bad_hi = 10004;
    read_through(5200, 10200);
    CHECK(laser_unreadable_blocks(1, 10000, 1000) == 4);
}

static void weak_zone_is_skipped(void)
{
    check_case("a weak zone, spots 160 blocks apart");
    fresh_disc();
    bad_mode = BAD_SPOTS;
    bad_lo = 20000;
    bad_hi = 21280;

    spot_failures = 0;
    read_through(19000, 23000);

    /* The first spot narrowed down, after landing again - five failed
     * reads, the failure, its second try and three narrowing down - and the
     * stretch after it skipped after one; then each spot reached skipped
     * after one failed read: 256 blocks cover the next one, so every other
     * spot. About ten in all, not the forty-odd of eight spots narrowed
     * down. */
    CHECK(spot_failures <= 12);
    CHECK(laser_unreadable_blocks(1, 19999, 1) == 0);
    CHECK(laser_unreadable_blocks(1, 20000, 4) == 4);
    /* At most as many blocks as the zone held lost past it. */
    CHECK(laser_unreadable_blocks(1, 21280 + 1280, 1) == 0);
}

static void blocks_sent_before_a_medium_error_are_returned(void)
{
    check_case("a medium error at the 17th block, the 16 before sent");
    fresh_disc();
    bad_mode = BAD_MEDIUM;
    bad_lo = 16;
    bad_hi = 17;
    sends_partial = 1;

    /* One command: the 16 blocks returned, 4 from the bad one recorded -
     * not landing again, the drive having landed, and no narrowing down. */
    CHECK(laser_read_blocks(1, 0, 32, buffer) == 16);
    CHECK(commands == 1);
    CHECK(laser_unreadable_blocks(1, 16, 32) == 4);
    CHECK(laser_unreadable_blocks(1, 15, 1) == 0);
}

int main(void)
{
    pthread_mutex_init(&entry.io_lock, NULL);
    pthread_mutex_init(&entry.learnt_lock, NULL);

    medium_error_is_narrowed_and_recorded();
    ranges_that_touch_are_merged();
    unanswered_read_is_recorded();
    unanswered_in_a_stream_is_skipped();
    other_failure_records_nothing();
    read_ahead_gives_up_at_once();
    read_ahead_stops_before_known_blocks();
    disc_change_forgets_everything();
    transfer_size_is_lowered_on_a_repeated_failure();
    transfer_size_survives_a_passing_failure();
    blocks_sent_before_a_stall_are_returned();
    failed_landing_is_read_after_landing_again();
    landing_again_is_once_a_read();
    lost_drive_is_restarted();
    read_ahead_does_not_land_again();
    damaged_stretch_is_skipped();
    dead_band_is_crossed();
    lone_bad_blocks_stay_narrow();
    weak_zone_is_skipped();
    blocks_sent_before_a_medium_error_are_returned();

    return check_result();
}
