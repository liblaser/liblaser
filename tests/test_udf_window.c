/* The read callback disc.c gives libudfread, against a simulated drive:
 * laser_read_blocks() and laser_read_blocks_ahead() are replaced by reads
 * that fill each block with its own address, refuse the blocks from a given
 * one on, and count the commands. libudfread itself is not run: its
 * functions are stubs, there for disc.c to link.
 *
 * What is tested is the window: which requests it serves without the
 * drive, and that it never costs a request the blocks it asked for. */

#include <assert.h>
#include <stdarg.h>

#include "../src/disc.c"

#include "check.h"

/* ---- the drive ---- */

static int      commands;        /* reads sent, of either kind */
static int      ahead_commands;  /* of which, as a read-ahead */

/* Blocks from this one on are refused. */
static uint32_t refused_from = UINT32_MAX;

/* Each block starts with its own address, and is otherwise filled with its
 * low byte. */
static void fill(uint32_t lba, int count, uint8_t *buffer)
{
    for (int i = 0; i < count; i++) {
        uint8_t *block = buffer + (size_t)i * UDF_BLOCK_BYTES;
        memset(block, (int)((lba + i) & 0xff), UDF_BLOCK_BYTES);
        block[0] = (uint8_t)((lba + i) >> 8);
    }
}

static int filled(const uint8_t *buffer, uint32_t lba, int count)
{
    for (int i = 0; i < count; i++) {
        const uint8_t *block = buffer + (size_t)i * UDF_BLOCK_BYTES;
        if (block[0] != (uint8_t)((lba + i) >> 8) ||
            block[1] != (uint8_t)((lba + i) & 0xff))
            return 0;
    }
    return 1;
}

/* As laser_read_blocks(): the blocks before a refused one are returned. */
int laser_read_blocks(int token, uint32_t lba, int count, uint8_t *buffer)
{
    (void) token;
    commands++;
    if (lba >= refused_from)
        return LASER_ERR_SCRAMBLED;
    if (lba + (uint32_t)count > refused_from)
        count = (int)(refused_from - lba);
    fill(lba, count, buffer);
    return count;
}

/* Simpler than laser_read_blocks_ahead(), which returns the chunks before
 * the one that fails: this one fails whole, the harder case for a window. */
int laser_read_blocks_ahead(int token, uint32_t lba, int count,
                            uint8_t *buffer)
{
    (void) token;
    commands++;
    ahead_commands++;
    if (lba + (uint32_t)count > refused_from)
        return LASER_ERR_SCRAMBLED;
    fill(lba, count, buffer);
    return count;
}

/* ---- the rest of what disc.c calls ---- */

laser_status_t laser_scsi_cdb(int token, const uint8_t *cdb, int cdb_len,
                              uint8_t *data, int data_len, int data_in,
                              int *actual_len)
{
    (void) token;
    (void) cdb;
    (void) cdb_len;
    (void) data;
    (void) data_len;
    (void) data_in;
    (void) actual_len;
    return LASER_ERR_IO;
}

void laser_log(laser_log_level_t level, const char *fmt, ...)
{
    (void) level;
    (void) fmt;
}

void laser_token_settle(int token)
{
    (void) token;
}

int laser_wait_state(int token)
{
    (void) token;
    return 0;
}

laser_family_t laser_medium_family(int token)
{
    (void) token;
    return LASER_FAMILY_UNKNOWN;
}

udfread *udfread_init(void)
{
    return NULL;
}

int udfread_open_input(udfread *udf, struct udfread_block_input *input)
{
    (void) udf;
    (void) input;
    return -1;
}

void udfread_close(udfread *udf)
{
    (void) udf;
}

const char *udfread_get_volume_id(udfread *udf)
{
    (void) udf;
    return NULL;
}

UDFDIR *udfread_opendir(udfread *udf, const char *path)
{
    (void) udf;
    (void) path;
    return NULL;
}

struct udfread_dirent *udfread_readdir(UDFDIR *dir,
                                       struct udfread_dirent *entry)
{
    (void) dir;
    (void) entry;
    return NULL;
}

void udfread_closedir(UDFDIR *dir)
{
    (void) dir;
}

UDFFILE *udfread_file_open(udfread *udf, const char *path)
{
    (void) udf;
    (void) path;
    return NULL;
}

void udfread_file_close(UDFFILE *file)
{
    (void) file;
}

int64_t udfread_file_size(UDFFILE *file)
{
    (void) file;
    return -1;
}

ssize_t udfread_file_read(UDFFILE *file, void *buffer, size_t bytes)
{
    (void) file;
    (void) buffer;
    (void) bytes;
    return -1;
}

/* ---- the cases ---- */

int main(void)
{
    static uint8_t window[UDF_WINDOW_BLOCKS * UDF_BLOCK_BYTES];
    static uint8_t buffer[64 * UDF_BLOCK_BYTES];

    struct scsi_block_input input = {
        .base   = { .read = udf_read_cb },
        .token  = 1,
        .window = window,
    };
    struct udfread_block_input *bi = &input.base;

    check_case("eight single blocks close together: one command");
    static const uint32_t walk[] = { 256, 257, 258, 272, 273, 260, 280, 287 };
    for (size_t i = 0; i < sizeof(walk) / sizeof(walk[0]); i++) {
        CHECK(udf_read_cb(bi, walk[i], buffer, 1, 0) == 1);
        CHECK(filled(buffer, walk[i], 1));
    }
    CHECK(commands == 1);

    check_case("requests past the window, and across its end: a window "
               "each");
    commands = 0;
    CHECK(udf_read_cb(bi, 300, buffer, 2, 0) == 2);
    CHECK(filled(buffer, 300, 2));
    CHECK(udf_read_cb(bi, 330, buffer, 4, 0) == 4);
    CHECK(filled(buffer, 330, 4));
    CHECK(commands == 2);

    check_case("a request of a window's size or more: read as asked");
    commands = 0;
    ahead_commands = 0;
    CHECK(udf_read_cb(bi, 1000, buffer, 40, 0) == 40);
    CHECK(filled(buffer, 1000, 40));
    CHECK(commands == 1);
    CHECK(ahead_commands == 0);

    check_case("a window that would reach refused blocks: the request is "
               "still read");
    refused_from = 2010;
    commands = 0;
    CHECK(udf_read_cb(bi, 2000, buffer, 1, 0) == 1);
    CHECK(filled(buffer, 2000, 1));
    CHECK(commands == 2);
    /* And a refused block is a refused block. */
    CHECK(udf_read_cb(bi, 2010, buffer, 1, 0) == -1);
    refused_from = UINT32_MAX;

    check_case("a file's own data: no window");
    commands = 0;
    ahead_commands = 0;
    input.file_data = true;
    CHECK(udf_read_cb(bi, 5000, buffer, 1, 0) == 1);
    CHECK(commands == 1);
    CHECK(ahead_commands == 0);
    input.file_data = false;

    check_case("no window at all: every request read as asked");
    input.window = NULL;
    commands = 0;
    ahead_commands = 0;
    CHECK(udf_read_cb(bi, 256, buffer, 1, 0) == 1);
    CHECK(commands == 1);
    CHECK(ahead_commands == 0);

    check_case("the walk's own count of its reads");
    CHECK(input.requests == 15);
    CHECK(input.commands == 10);

    return check_result();
}
