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
} bad_mode;
static uint32_t bad_lo, bad_hi;

/* A read not answered has sent the blocks before the bad one. */
static int sends_partial;

/* A bridge that fails every transfer above this many bytes; 0 for none. */
static int bridge_limit;

/* The next transfers above 32768 bytes to fail, in passing. */
static int transient_failures;

static int commands;

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

    if (bad_mode != BAD_NONE && lba < bad_hi && lba + count > bad_lo) {
        if (bad_mode == BAD_MEDIUM)
            why->cause = LASER_FAILURE_MEDIUM;
        if (bad_mode == BAD_NO_ANSWER)
            why->cause = LASER_FAILURE_NO_ANSWER;
        if (sends_partial && lba < bad_lo)
            why->received = (int)(bad_lo - lba) * 2048;
        return LASER_ERR_IO;
    }

    *actual_len = data_len;
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
    entry.largest_ok_bytes = 65536;
    entry.max_transfer_bytes = 65536;
    entry.cap_strike_bytes = 0;
    bad_mode = BAD_NONE;
    sends_partial = 0;
    bridge_limit = 0;
    transient_failures = 0;
    commands = 0;
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
    CHECK(laser_read_blocks(1, 0, 128, buffer) == 100);
    CHECK(recorded_ranges() == 1);
    CHECK(laser_unreadable_blocks(1, 100, 128) == 4);

    /* Asked for again, it is answered without the drive. */
    commands = 0;
    CHECK(laser_read_blocks(1, 100, 28, buffer) == LASER_ERR_IO);
    CHECK(commands == 0);
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
}

static void unanswered_read_is_recorded(void)
{
    check_case("a read the drive does not answer, while ready");
    fresh_disc();
    bad_mode = BAD_NO_ANSWER;
    bad_lo = 40;
    bad_hi = 41;

    CHECK(laser_read_blocks(1, 32, 32, buffer) == LASER_ERR_IO);
    CHECK(recorded_ranges() == 1);
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

    const unsigned generation = entry.media_generation;
    laser_forget_disc(&entry, 1);
    CHECK(recorded_ranges() == 0);
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

    /* The 16 blocks it sent are returned, the rest recorded. */
    CHECK(laser_read_blocks(1, 0, 32, buffer) == 16);
    CHECK(commands == 1);

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

int main(void)
{
    pthread_mutex_init(&entry.io_lock, NULL);
    pthread_mutex_init(&entry.learnt_lock, NULL);

    medium_error_is_narrowed_and_recorded();
    ranges_that_touch_are_merged();
    unanswered_read_is_recorded();
    other_failure_records_nothing();
    read_ahead_gives_up_at_once();
    read_ahead_stops_before_known_blocks();
    disc_change_forgets_everything();
    transfer_size_is_lowered_on_a_repeated_failure();
    transfer_size_survives_a_passing_failure();
    blocks_sent_before_a_stall_are_returned();

    return check_result();
}
