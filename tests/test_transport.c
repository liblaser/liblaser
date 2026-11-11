/* bot.c, scsi.c and medium.c against a scripted Bulk-Only device, in place
 * of libusb: each command block sent is handed to the case's scenario, which
 * queues what the device sends back - data, a stall, a status - or nothing,
 * for a device that does not answer.
 *
 * What is tested is what the transport and the command layer make of a
 * device's answers: the framing they accept, the sense they read, when they
 * send a command again and when they give up, and why. */

#include <assert.h>
#include <string.h>
#include <unistd.h>

/* The waits between two attempts are not what is tested. */
#define usleep(us) ((void)(us))

#include "../src/bot.c"
#include "../src/scsi.c"
#include "../src/medium.c"

#include "check.h"

static laser_entry_t entry;

/* ---- the device ---- */

/* What the device has queued for the host to read, in order. */
enum { PACKET_DATA, PACKET_STALL };
static struct {
    int     kind;
    int     len;
    uint8_t bytes[4096];
} queue[16];
static int queue_head, queue_tail;

/* The last command block received, and how many of each opcode since the
 * case began. */
static uint8_t  last_cdb[16];
static int      last_cdb_len;
static uint32_t last_tag;
static int      sent[256];

/* The data of a data-out command, when the scenario expects one. */
static int     expects_data_out;
static uint8_t data_out[64];
static int     data_out_len;

static int resets;

/* Called for each command block, with how many of its opcode that makes. */
static void (*scenario)(const uint8_t *cdb, int nth);

static void queue_bytes(const void *bytes, int len)
{
    queue[queue_tail].kind = PACKET_DATA;
    queue[queue_tail].len = len;
    if (len > 0)
        memcpy(queue[queue_tail].bytes, bytes, (size_t)len);
    queue_tail++;
}

static void queue_data(int len)
{
    static const uint8_t zeros[4096];
    queue_bytes(zeros, len);
}

static void queue_stall(void)
{
    queue[queue_tail++].kind = PACKET_STALL;
}

/* The status of the command: 0 passed, 1 failed. */
static void queue_status(int status)
{
    const bot_csw_t csw = { USB_BOT_CSW_SIGNATURE, last_tag, 0,
                            (uint8_t)status };
    queue_bytes(&csw, USB_BOT_CSW_SIZE);
}

/* The answer to REQUEST SENSE: fixed-format sense data under @p code -
 * 70h current, 71h deferred, with bit 7 for a valid @p lba. */
static void queue_sense_at(int code, int key, int asc, int ascq, uint32_t lba)
{
    uint8_t sense[18] = { 0 };
    sense[0]  = (uint8_t)code;
    sense[2]  = (uint8_t)key;
    sense[3]  = (uint8_t)(lba >> 24);
    sense[4]  = (uint8_t)(lba >> 16);
    sense[5]  = (uint8_t)(lba >> 8);
    sense[6]  = (uint8_t)lba;
    sense[7]  = 10;
    sense[12] = (uint8_t)asc;
    sense[13] = (uint8_t)ascq;
    queue_bytes(sense, sizeof(sense));
    queue_status(0);
}

static void queue_sense(int code, int key, int asc, int ascq)
{
    queue_sense_at(code, key, asc, ascq, 0);
}

/* A command failed the way a device fails a read: the data endpoint
 * stalled, then a failed status. */
static void queue_failed_read(void)
{
    queue_stall();
    queue_status(1);
}

/* ---- libusb, and what the three files call outside themselves ---- */

int libusb_bulk_transfer(libusb_device_handle *handle, unsigned char endpoint,
                         unsigned char *data, int length, int *transferred,
                         unsigned int timeout)
{
    (void) handle;
    (void) timeout;

    const int out = !(endpoint & 0x80);

    if (out && expects_data_out) {
        data_out_len = length;
        memcpy(data_out, data, (size_t)length < sizeof(data_out)
                               ? (size_t)length : sizeof(data_out));
        expects_data_out = 0;
        *transferred = length;
        return 0;
    }

    if (out) {
        /* A command block wrapper: whatever the device had queued for the
         * command before is gone. */
        bot_cbw_t cbw;
        assert(length == USB_BOT_CBW_SIZE);
        memcpy(&cbw, data, sizeof(cbw));
        last_tag = cbw.dCBWTag;
        last_cdb_len = cbw.bCBWCBLength;
        memcpy(last_cdb, cbw.CBWCB, sizeof(last_cdb));
        queue_head = queue_tail = 0;
        *transferred = length;
        scenario(cbw.CBWCB, ++sent[cbw.CBWCB[0]]);
        return 0;
    }

    if (queue_head == queue_tail) {
        *transferred = 0;
        return LIBUSB_ERROR_TIMEOUT;
    }
    if (queue[queue_head].kind == PACKET_STALL) {
        queue_head++;
        *transferred = 0;
        return LIBUSB_ERROR_PIPE;
    }

    const int len = queue[queue_head].len < length ? queue[queue_head].len
                                                   : length;
    memcpy(data, queue[queue_head].bytes, (size_t)len);
    queue_head++;
    *transferred = len;
    return 0;
}

int libusb_clear_halt(libusb_device_handle *handle, unsigned char endpoint)
{
    (void) handle;
    (void) endpoint;
    return 0;
}

const char *libusb_error_name(int code)
{
    (void) code;
    return "error";
}

void laser_mass_storage_reset(laser_entry_t *e)
{
    (void) e;
    resets++;
    queue_head = queue_tail = 0;
}

laser_entry_t *laser_pin(int token)
{
    (void) token;
    return &entry;
}

void laser_unpin(laser_entry_t *e)
{
    (void) e;
}

int laser_is_cancelled(const laser_entry_t *e)
{
    (void) e;
    return 0;
}

int laser_css_session_is_open(laser_entry_t *e)
{
    (void) e;
    return 1;
}

int laser_cdb_changes_css_state(const uint8_t *cdb, int cdb_len)
{
    (void) cdb;
    (void) cdb_len;
    return 0;
}

void laser_disc_changed_locked(laser_entry_t *e)
{
    e->media_generation++;
}

void laser_forget_disc(laser_entry_t *e, int token)
{
    (void) e;
    (void) token;
}

void laser_log(laser_log_level_t level, const char *fmt, ...)
{
    (void) level;
    (void) fmt;
}

/* ---- helpers ---- */

#define OP_TEST_UNIT_READY   0x00
#define OP_REQUEST_SENSE     0x03
#define OP_START_STOP_UNIT   0x1b
#define OP_READ_10           0x28
#define OP_GET_CONFIGURATION 0x46
#define OP_SET_STREAMING     0xb6
#define OP_SET_CD_SPEED      0xbb

static void begin(const char *name, void (*s)(const uint8_t *, int))
{
    check_case(name);
    scenario = s;
    memset(sent, 0, sizeof(sent));
    resets = 0;
    entry.device_gone = 0;
    entry.wait_state = 0;
    entry.unanswered_count = 0;
}

/* One block at LBA 100, read as a reader does: given up after two medium
 * errors, or two attempts not answered. */
static uint8_t block[2048];

static laser_status_t read_block(laser_failure_t *why, int max_resets)
{
    static const uint8_t cdb[10] = { OP_READ_10, 0, 0, 0, 0, 100, 0, 0, 1, 0 };
    int actual = 0;
    return laser_scsi_cdb_read(1, cdb, sizeof(cdb), block, sizeof(block), 1,
                               &actual, 2, max_resets, why);
}

/* ---- scenarios ---- */

static void device_answers(const uint8_t *cdb, int nth)
{
    (void) nth;
    if (cdb[0] == OP_READ_10)
        queue_data(2048);
    queue_status(0);
}

/* A device sending its failed status where the data was expected. */
static void status_in_place_of_data(const uint8_t *cdb, int nth)
{
    (void) nth;
    if (cdb[0] == OP_GET_CONFIGURATION)
        queue_status(1);
    else if (cdb[0] == OP_REQUEST_SENSE)
        queue_sense(0x70, 5, 0x20, 0);
}

/* A device ending its data with an empty packet. */
static void empty_packet_before_status(const uint8_t *cdb, int nth)
{
    (void) cdb;
    (void) nth;
    queue_data(2048);
    queue_data(0);
    queue_status(0);
}

/* The sense the next two scenarios answer REQUEST SENSE with. */
static int sense_code = 0x70, sense_key, sense_asc, sense_ascq;

/* Every read fails. */
static void reads_fail(const uint8_t *cdb, int nth)
{
    (void) nth;
    if (cdb[0] == OP_READ_10)
        queue_failed_read();
    else if (cdb[0] == OP_REQUEST_SENSE)
        queue_sense(sense_code, sense_key, sense_asc, sense_ascq);
    else
        queue_status(0);
}

/* The first read fails, the next ones do not. */
static void first_read_fails(const uint8_t *cdb, int nth)
{
    if (cdb[0] == OP_READ_10 && nth > 1) {
        queue_data(2048);
        queue_status(0);
    } else {
        reads_fail(cdb, nth);
    }
}

/* Every command but REQUEST SENSE fails. */
static void commands_fail(const uint8_t *cdb, int nth)
{
    (void) nth;
    if (cdb[0] == OP_REQUEST_SENSE)
        queue_sense(sense_code, sense_key, sense_asc, sense_ascq);
    else
        queue_status(1);
}

/* A drive stuck on the sector: no answer to a read, ready otherwise. */
static void reads_unanswered(const uint8_t *cdb, int nth)
{
    (void) nth;
    if (cdb[0] != OP_READ_10)
        queue_status(0);
}

/* A drive spinning up: no answer to the first read, "becoming ready" to the
 * TEST UNIT READY that follows, then itself again. */
static void spinning_up(const uint8_t *cdb, int nth)
{
    if (cdb[0] == OP_READ_10) {
        if (nth >= 2) {
            queue_data(2048);
            queue_status(0);
        }
    } else if (cdb[0] == OP_TEST_UNIT_READY) {
        queue_status(nth == 1 ? 1 : 0);
    } else if (cdb[0] == OP_REQUEST_SENSE) {
        queue_sense(0x70, 2, 4, 1);
    }
}

/* A drive that goes on trying a sector it was given up on: silent to the
 * first read and to every TEST UNIT READY, then failing the second read
 * with deferred sense - @c deferred_key, naming @c deferred_lba if
 * @c deferred_lba_valid - and reading from the third on. */
static int      deferred_key, deferred_lba_valid;
static uint32_t deferred_lba;

static void retries_then_deferred_sense(const uint8_t *cdb, int nth)
{
    if (cdb[0] == OP_READ_10 && nth == 2) {
        queue_failed_read();
    } else if (cdb[0] == OP_READ_10 && nth > 2) {
        queue_data(2048);
        queue_status(0);
    } else if (cdb[0] == OP_REQUEST_SENSE) {
        queue_sense_at(0x71 | (deferred_lba_valid ? 0x80 : 0), deferred_key,
                       0x11, 0, deferred_lba);
    }
}

/* The same drive, having sent half of the block before going silent. */
static void half_a_block_then_silence(const uint8_t *cdb, int nth)
{
    if (cdb[0] == OP_READ_10 && nth == 1)
        queue_data(1024);
    else
        retries_then_deferred_sense(cdb, nth);
}

/* A drive holding a DVD, taking SET STREAMING or refusing it. */
static int takes_set_streaming;

static void drive_with_a_dvd(const uint8_t *cdb, int nth)
{
    (void) nth;
    if (cdb[0] == OP_GET_CONFIGURATION) {
        uint8_t header[32] = { 0 };
        header[7] = 0x10;                   /* current profile: DVD-ROM */
        queue_bytes(header, sizeof(header));
        queue_status(0);
    } else if (cdb[0] == OP_SET_STREAMING) {
        expects_data_out = 1;
        queue_status(takes_set_streaming ? 0 : 1);
    } else if (cdb[0] == OP_REQUEST_SENSE) {
        queue_sense(0x70, 5, 0x20, 0);
    } else {
        queue_status(0);
    }
}

/* ---- the cases ---- */

static void framing(void)
{
    laser_failure_t why;
    int actual = 0;

    begin("a read, its command block as long as the command", device_answers);
    CHECK(read_block(&why, 2) == LASER_OK);
    CHECK(last_cdb_len == 10);

    begin("the same on an ATAPI-subclass interface, padded to 12 bytes",
          device_answers);
    entry.pad_cdb12 = 1;
    CHECK(read_block(&why, 2) == LASER_OK);
    CHECK(last_cdb_len == 12);
    entry.pad_cdb12 = 0;

    begin("a status sent in place of the data", status_in_place_of_data);
    static const uint8_t get_configuration[10] =
        { OP_GET_CONFIGURATION, 0, 0, 0, 0, 0, 0, 0, 32, 0 };
    CHECK(laser_scsi_cdb(1, get_configuration, sizeof(get_configuration),
                         block, 32, 1, &actual) == LASER_ERR_REFUSED);
    CHECK(resets == 0);
    CHECK(sent[OP_GET_CONFIGURATION] == 1);

    begin("an empty packet between the data and the status",
          empty_packet_before_status);
    CHECK(read_block(&why, 2) == LASER_OK);
    CHECK(resets == 0);
}

static void sense(void)
{
    laser_failure_t why;
    sense_code = 0x70;

    begin("BLANK CHECK: refused, not read again", reads_fail);
    sense_key = 8, sense_asc = 0, sense_ascq = 0;
    CHECK(read_block(&why, 2) == LASER_ERR_REFUSED);
    CHECK(sent[OP_READ_10] == 1);

    begin("NOT READY, start needed: the drive is started, the read sent again",
          first_read_fails);
    sense_key = 2, sense_asc = 4, sense_ascq = 2;
    CHECK(read_block(&why, 2) == LASER_OK);
    CHECK(sent[OP_START_STOP_UNIT] == 1);
    CHECK(sent[OP_READ_10] == 2);

    begin("NOT READY, manual intervention: given up at once", reads_fail);
    sense_key = 2, sense_asc = 4, sense_ascq = 3;
    CHECK(read_block(&why, 2) == LASER_ERR_IO);
    CHECK(sent[OP_READ_10] == 1);
    CHECK(why.cause == LASER_FAILURE_OTHER);

    begin("HARDWARE ERROR: read again once, and not the disc's fault",
          reads_fail);
    sense_key = 4, sense_asc = 9, sense_ascq = 0;
    CHECK(read_block(&why, 2) == LASER_ERR_IO);
    CHECK(sent[OP_READ_10] == 2);
    CHECK(why.cause != LASER_FAILURE_MEDIUM);

    begin("MEDIUM ERROR: read again once, and the disc's fault", reads_fail);
    sense_key = 3, sense_asc = 0x11, sense_ascq = 5;
    CHECK(read_block(&why, 2) == LASER_ERR_IO);
    CHECK(sent[OP_READ_10] == 2);
    CHECK(why.cause == LASER_FAILURE_MEDIUM);
}

static void deferred_sense(void)
{
    laser_failure_t why;
    uint8_t key, asc, ascq;

    begin("deferred sense of an earlier command: this one is sent again",
          first_read_fails);
    sense_code = 0x71;
    sense_key = 5, sense_asc = 0x21, sense_ascq = 0;
    CHECK(read_block(&why, 2) == LASER_OK);
    CHECK(sent[OP_READ_10] == 2);

    begin("no medium, as deferred sense: the state of the unit, read as "
          "this command's", reads_fail);
    sense_key = 2, sense_asc = 0x3a, sense_ascq = 1;
    CHECK(read_block(&why, 2) == LASER_ERR_MEDIA_GONE);
    CHECK(sent[OP_READ_10] == 1);

    begin("tray open, as deferred sense, to TEST UNIT READY", commands_fail);
    sense_ascq = 2;
    CHECK(laser_test_unit_ready_locked(&entry, &key, &asc, &ascq) != 0);
    CHECK(key == 2 && asc == 0x3a && ascq == 2);

    sense_code = 0x70;
}

static void unanswered_reads(void)
{
    laser_failure_t why;

    begin("a read not answered by a drive that says it is ready",
          reads_unanswered);
    CHECK(read_block(&why, 2) == LASER_ERR_IO);
    CHECK(sent[OP_READ_10] == 1);
    CHECK(why.cause == LASER_FAILURE_NO_ANSWER);

    begin("a read not answered by a drive spinning up: waited for",
          spinning_up);
    CHECK(read_block(&why, 2) == LASER_OK);
    CHECK(sent[OP_READ_10] == 2);

    begin("not answered, then a deferred MEDIUM ERROR: the read's answer",
          retries_then_deferred_sense);
    deferred_key = 3, deferred_lba_valid = 0;
    CHECK(read_block(&why, 2) == LASER_ERR_IO);
    CHECK(sent[OP_READ_10] == 2);
    CHECK(why.cause == LASER_FAILURE_NO_ANSWER);
    CHECK(why.sense_key == 3);
    CHECK(why.received == 0);

    begin("the same, naming the block read", retries_then_deferred_sense);
    deferred_lba_valid = 1, deferred_lba = 100;
    CHECK(read_block(&why, 2) == LASER_ERR_IO);
    CHECK(sent[OP_READ_10] == 2);
    CHECK(why.cause == LASER_FAILURE_NO_ANSWER);

    begin("the same, naming another block: another read's, this one is "
          "sent again", retries_then_deferred_sense);
    deferred_lba = 5000;
    CHECK(read_block(&why, 2) == LASER_OK);
    CHECK(sent[OP_READ_10] == 3);

    begin("not answered, then a deferred HARDWARE ERROR: sent again",
          retries_then_deferred_sense);
    deferred_key = 4, deferred_lba_valid = 0;
    CHECK(read_block(&why, 2) == LASER_OK);
    CHECK(sent[OP_READ_10] == 3);

    begin("a read-ahead not answered: one attempt, nobody blamed",
          retries_then_deferred_sense);
    deferred_key = 3;
    CHECK(read_block(&why, 1) == LASER_ERR_IO);
    CHECK(sent[OP_READ_10] == 1);
    CHECK(why.cause == LASER_FAILURE_OTHER);

    /* No begin(): the device goes on from the read-ahead above. */
    check_case("then the same blocks read: the deferred MEDIUM ERROR is "
               "the answer at once");
    CHECK(read_block(&why, 2) == LASER_ERR_IO);
    CHECK(sent[OP_READ_10] == 2);
    CHECK(why.cause == LASER_FAILURE_NO_ANSWER);

    begin("a read the drive stops answering after 1024 bytes",
          half_a_block_then_silence);
    CHECK(read_block(&why, 2) == LASER_ERR_IO);
    CHECK(why.received == 1024);
}

static void speed(void)
{
    begin("2x asked of a drive holding a DVD", drive_with_a_dvd);
    takes_set_streaming = 1;
    CHECK(laser_set_speed(1, 2) == LASER_OK);
    /* One performance descriptor: 2770 kB each 1000 ms, not the drive's
     * defaults. */
    const unsigned kb = (unsigned)data_out[12] << 24 | data_out[13] << 16 |
                        data_out[14] << 8 | data_out[15];
    const unsigned ms = data_out[18] << 8 | data_out[19];
    CHECK(data_out_len == 28 && last_cdb[10] == 28);
    CHECK(kb == 2770 && ms == 1000);
    CHECK(!(data_out[0] & 0x04));
    CHECK(sent[OP_SET_CD_SPEED] == 0);

    begin("the speed given back", drive_with_a_dvd);
    CHECK(laser_set_speed(1, 0) == LASER_OK);
    CHECK(data_out[0] & 0x04);
    CHECK(sent[OP_GET_CONFIGURATION] == 0);

    begin("SET STREAMING refused: SET CD SPEED instead", drive_with_a_dvd);
    takes_set_streaming = 0;
    CHECK(laser_set_speed(1, 2) == LASER_OK);
    CHECK(sent[OP_SET_CD_SPEED] == 1);
    CHECK((last_cdb[2] << 8 | last_cdb[3]) == 2770);
}

int main(void)
{
    pthread_mutex_init(&entry.io_lock, NULL);
    entry.ep_in = 0x81;
    entry.ep_out = 0x02;
    entry.handle = (void *)&entry;

    framing();
    sense();
    deferred_sense();
    unanswered_reads();
    speed();

    return check_result();
}
