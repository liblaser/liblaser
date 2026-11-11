/* disc.c, whole, against discs that are image files: laser_read_blocks()
 * and laser_read_blocks_ahead() read the image, and the drive's other
 * answers - what kind of medium it holds, its table of contents, whether it
 * is ready - are each case's. libudfread is the real one, linked in: what
 * is tested is that an identification finds its way through a UDF volume
 * as a DVD and a Blu-ray lay it out, and through the ISO 9660 of a Video
 * CD, to the answer.
 *
 * The images are in tests/images, with the script that made them, which
 * says what each holds. The directory is the program's argument. */

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

#include "../src/disc.c"

#include "check.h"

#define BLOCK 2048

/* ---- the drive ---- */

/* The disc: an image, NULL for none. */
static uint8_t *image;
static uint32_t image_blocks;

/* What the drive says it holds, and whether it is ready. */
static laser_family_t family;
static int            wait_state;

/* Its table of contents: whether there is one, the first track's control
 * field - 4 for data, 0 for audio - and where the first data track starts. */
static int      toc_present;
static uint8_t  toc_control;
static uint32_t toc_track_lba;

static int commands;

static int read_image(uint32_t lba, int count, uint8_t *buffer)
{
    commands++;
    if (image == NULL || lba >= image_blocks)
        return LASER_ERR_IO;
    if ((uint32_t)count > image_blocks - lba)
        count = (int)(image_blocks - lba);
    memcpy(buffer, image + (size_t)lba * BLOCK, (size_t)count * BLOCK);
    return count;
}

int laser_read_blocks(int token, uint32_t lba, int count, uint8_t *buffer)
{
    (void) token;
    return read_image(lba, count, buffer);
}

int laser_read_blocks_ahead(int token, uint32_t lba, int count,
                            uint8_t *buffer)
{
    (void) token;
    return read_image(lba, count, buffer);
}

/* READ TOC, the one command disc.c sends by itself: a first track and the
 * lead-out. */
laser_status_t laser_scsi_cdb(int token, const uint8_t *cdb, int cdb_len,
                              uint8_t *data, int data_len, int data_in,
                              int *actual_len)
{
    (void) token;
    (void) cdb_len;
    (void) data_in;

    if (cdb[0] != 0x43 || !toc_present || data_len < 20)
        return LASER_ERR_IO;

    memset(data, 0, (size_t)data_len);
    data[1] = 18;                   /* the bytes that follow */
    data[2] = data[3] = 1;          /* first and last track */
    data[5] = toc_control;
    data[6] = 1;
    data[8]  = (uint8_t)(toc_track_lba >> 24);
    data[9]  = (uint8_t)(toc_track_lba >> 16);
    data[10] = (uint8_t)(toc_track_lba >> 8);
    data[11] = (uint8_t)toc_track_lba;
    data[13] = toc_control;
    data[14] = 0xAA;
    *actual_len = 20;
    return LASER_OK;
}

laser_family_t laser_medium_family(int token)
{
    (void) token;
    return family;
}

void laser_token_settle(int token)
{
    (void) token;
}

int laser_wait_state(int token)
{
    (void) token;
    return wait_state;
}

void laser_log(laser_log_level_t level, const char *fmt, ...)
{
    (void) level;
    (void) fmt;
}

/* What libudfread's udfread_open() opens a device with, by its path: in
 * default_blockinput.c, which the vendored build leaves out, disc.c giving
 * libudfread its own input. Nothing here calls udfread_open(), but it is in
 * the object file that has everything else, and has to link. */
struct udfread_block_input *block_input_new(void);
struct udfread_block_input *block_input_new(void)
{
    return NULL;
}

/* ---- the cases ---- */

static const char *image_dir;

/* Puts the image @p name in the drive, NULL for none, as a medium of
 * @p medium that is ready. */
static void load(const char *name, laser_family_t medium)
{
    free(image);
    image = NULL;
    image_blocks = 0;
    family = medium;
    wait_state = LASER_WAIT_READY;
    toc_present = 0;
    toc_control = 0;
    toc_track_lba = 0;
    commands = 0;

    if (name == NULL)
        return;

    char path[1024];
    snprintf(path, sizeof(path), "%s/%s", image_dir, name);
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        printf("    cannot open %s\n", path);
        check_failures++;
        return;
    }
    fseek(f, 0, SEEK_END);
    const long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    image = malloc((size_t)size);
    if (image == NULL || fread(image, 1, (size_t)size, f) != (size_t)size) {
        printf("    cannot read %s\n", path);
        check_failures++;
        free(image);
        image = NULL;
    } else {
        image_blocks = (uint32_t)(size / BLOCK);
    }
    fclose(f);
}

/* A CD's table of contents: a data track starting at @p track_lba, or with
 * @p control 0 an audio one. */
static void toc(uint8_t control, uint32_t track_lba)
{
    toc_present = 1;
    toc_control = control;
    toc_track_lba = track_lba;
}

static laser_disc_t identify(const char *const *langs)
{
    laser_disc_t disc;
    laser_disc_identify_lang(1, langs, &disc);
    return disc;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s <directory of the images>\n", argv[0]);
        return 2;
    }
    image_dir = argv[1];

    laser_disc_t disc;

    check_case("a DVD-Video, its /AUDIO_TS empty");
    load("dvd-video.img", LASER_FAMILY_DVD);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_DVD_VIDEO);
    CHECK(disc.state == LASER_DISC_STATE_PRESENT);
    CHECK(strcmp(disc.volume_id, "LASER_DVD") == 0);
    CHECK(disc.title[0] == '\0');
    /* The whole walk in a few windows: the structures are together. */
    CHECK(commands <= 8);

    check_case("a DVD-Audio");
    load("dvd-audio.img", LASER_FAMILY_DVD);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_DVD_AUDIO);
    CHECK(strcmp(disc.volume_id, "LASER_DVDA") == 0);

    check_case("a universal disc: both zones");
    load("dvd-universal.img", LASER_FAMILY_DVD);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_DVD_UNIVERSAL);
    CHECK(strcmp(disc.volume_id, "LASER_BOTH") == 0);

    check_case("a DVD whose VIDEO_TS.IFO is not a video manager: not a "
               "DVD-Video");
    load("udf-data.img", LASER_FAMILY_DVD);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_UNKNOWN);
    CHECK(disc.state == LASER_DISC_STATE_PRESENT);
    CHECK(disc.volume_id[0] == '\0');

    check_case("a Blu-ray: directories in a metadata partition, the title "
               "in English when no language is asked for");
    load("bd-video.img", LASER_FAMILY_BD);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_BD_VIDEO);
    CHECK(strcmp(disc.volume_id, "LASER_BD") == 0);
    CHECK(strcmp(disc.title, "Laser test disc") == 0);

    check_case("the same, in French: a file recorded in its entry, its "
               "character references decoded");
    load("bd-video.img", LASER_FAMILY_BD);
    disc = identify((const char *const[]){ "fra", NULL });
    CHECK(disc.kind == LASER_DISC_BD_VIDEO);
    CHECK(strcmp(disc.title, "Disque d'essai \xc3\xa9t\xc3\xa9") == 0);

    check_case("the same, in German: a placeholder for a title, passed "
               "over for the English one");
    load("bd-video.img", LASER_FAMILY_BD);
    disc = identify((const char *const[]){ "deu", NULL });
    CHECK(strcmp(disc.title, "Laser test disc") == 0);

    check_case("a Blu-ray the drive gives no profile for: found all the "
               "same");
    load("bd-video.img", LASER_FAMILY_UNKNOWN);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_BD_VIDEO);

    check_case("a Video CD");
    load("vcd.iso", LASER_FAMILY_CD);
    toc(4, 0);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_VCD);
    CHECK(strcmp(disc.volume_id, "LASER_VCD") == 0);
    /* The descriptor, two directories, the file: no UDF looked for. */
    CHECK(commands == 4);

    check_case("a Super Video CD, its filesystem in a track starting at "
               "sector 75");
    load("svcd-track2.iso", LASER_FAMILY_CD);
    toc(4, 75);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_SVCD);
    CHECK(strcmp(disc.volume_id, "LASER_SVCD") == 0);

    check_case("the same, the track looked for at sector 0: nothing "
               "recognised");
    load("svcd-track2.iso", LASER_FAMILY_CD);
    toc(4, 0);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_UNKNOWN);

    check_case("a Video CD the drive gives no profile for: found after "
               "the UDF it does not have");
    load("vcd.iso", LASER_FAMILY_UNKNOWN);
    toc(4, 0);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_VCD);

    check_case("an audio CD: its table of contents is enough");
    load(NULL, LASER_FAMILY_CD);
    toc(0, 0);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_CD_AUDIO);
    CHECK(commands == 0);

    check_case("a data CD that is no Video CD");
    load("udf-data.img", LASER_FAMILY_CD);
    toc(4, 0);
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_UNKNOWN);
    CHECK(disc.state == LASER_DISC_STATE_PRESENT);

    check_case("an empty drive: nothing read");
    load("dvd-video.img", LASER_FAMILY_DVD);
    wait_state = LASER_WAIT_NO_MEDIUM;
    disc = identify(NULL);
    CHECK(disc.kind == LASER_DISC_UNKNOWN);
    CHECK(disc.state == LASER_DISC_STATE_NO_MEDIUM);
    CHECK(commands == 0);

    check_case("a tray open");
    load(NULL, LASER_FAMILY_UNKNOWN);
    wait_state = LASER_WAIT_TRAY_OPEN;
    disc = identify(NULL);
    CHECK(disc.state == LASER_DISC_STATE_TRAY_OPEN);
    CHECK(commands == 0);

    free(image);
    return check_result();
}
