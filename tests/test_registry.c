/* registry.c with libusb and the bringing up of a drive replaced by
 * functions that succeed at once: three threads pin the entry of a token
 * and use it, as an operation does, while the main thread acquires and
 * releases the token's only claim - registering the device and tearing it
 * down - three thousand times.
 *
 * What is tested is that a teardown waits for the operations in flight: no
 * thread may ever find, under its pin, an entry that is another device's or
 * nobody's. */

#include <assert.h>
#include <stdatomic.h>
#include <unistd.h>

#include "../src/registry.c"

#include "check.h"

/* ---- libusb ---- */

static int fake_context, fake_handle;

int libusb_init_context(libusb_context **ctx,
                        const struct libusb_init_option options[],
                        int num_options)
{
    (void) options;
    (void) num_options;
    *ctx = (void *)&fake_context;
    return 0;
}

int libusb_wrap_sys_device(libusb_context *ctx, intptr_t fd,
                           libusb_device_handle **handle)
{
    (void) ctx;
    (void) fd;
    *handle = (void *)&fake_handle;
    return 0;
}

libusb_device *libusb_get_device(libusb_device_handle *handle)
{
    return (void *)handle;
}

int libusb_get_device_descriptor(libusb_device *dev,
                                 struct libusb_device_descriptor *desc)
{
    (void) dev;
    memset(desc, 0, sizeof(*desc));
    return 0;
}

int libusb_get_device_speed(libusb_device *dev)
{
    (void) dev;
    return LIBUSB_SPEED_HIGH;
}

int libusb_kernel_driver_active(libusb_device_handle *handle, int iface)
{
    (void) handle;
    (void) iface;
    return 0;
}

int libusb_detach_kernel_driver(libusb_device_handle *handle, int iface)
{
    (void) handle;
    (void) iface;
    return 0;
}

int libusb_attach_kernel_driver(libusb_device_handle *handle, int iface)
{
    (void) handle;
    (void) iface;
    return 0;
}

int libusb_claim_interface(libusb_device_handle *handle, int iface)
{
    (void) handle;
    (void) iface;
    return 0;
}

int libusb_release_interface(libusb_device_handle *handle, int iface)
{
    (void) handle;
    (void) iface;
    return 0;
}

void libusb_close(libusb_device_handle *handle)
{
    (void) handle;
}

void libusb_exit(libusb_context *ctx)
{
    (void) ctx;
}

const char *libusb_error_name(int code)
{
    (void) code;
    return "error";
}

/* ---- the bringing up of a drive: usb.c, ready.c, medium.c ---- */

int laser_find_bulk_endpoints(laser_entry_t *entry)
{
    (void) entry;
    return 0;
}

void laser_mass_storage_reset(laser_entry_t *entry)
{
    (void) entry;
}

int laser_probe_lun(laser_entry_t *entry)
{
    (void) entry;
    return LASER_OPTICAL_YES;
}

void laser_allow_medium_removal(laser_entry_t *entry)
{
    (void) entry;
}

int laser_wait_until_ready(laser_entry_t *entry)
{
    (void) entry;
    return 0;
}

void laser_drop_media_events(laser_entry_t *entry)
{
    (void) entry;
}

/* ---- the test ---- */

static void quiet(void *opaque, laser_log_level_t level, const char *message)
{
    (void) opaque;
    (void) level;
    (void) message;
}

static atomic_int stop;
static atomic_int pinned;       /* pins that found the device registered */
static atomic_int refused;      /* pins that found it torn down */
static atomic_int torn;         /* entries found torn down under a pin */

static void *operation(void *arg)
{
    const int token = *(int *)arg;

    while (!atomic_load(&stop)) {
        laser_entry_t *entry = laser_pin(token);
        if (entry == NULL) {
            atomic_fetch_add(&refused, 1);
            continue;
        }

        /* As an operation does: the transport under io_lock, more than
         * once, with the lock given up in between. */
        for (int i = 0; i < 3; i++) {
            pthread_mutex_lock(&entry->io_lock);
            if (entry->handle != (void *)&fake_handle ||
                entry->token != token || !entry->in_use)
                atomic_fetch_add(&torn, 1);
            pthread_mutex_unlock(&entry->io_lock);
            usleep(50);
        }

        atomic_fetch_add(&pinned, 1);
        laser_unpin(entry);
    }
    return NULL;
}

int main(void)
{
    laser_set_log_cb(quiet, NULL);

    /* Standard input: a descriptor fstat() works on, which is all the
     * registry asks of one here. */
    int token = 0;

    check_case("three operations pinning an entry that is registered and "
               "torn down 3000 times");

    pthread_t threads[3];
    for (int i = 0; i < 3; i++)
        pthread_create(&threads[i], NULL, operation, &token);

    int acquired = 0;
    for (int i = 0; i < 3000; i++) {
        if (laser_acquire(token) == LASER_OK)
            acquired++;
        usleep(100);
        laser_release(token);       /* the only claim: tears down */
    }

    atomic_store(&stop, 1);
    for (int i = 0; i < 3; i++)
        pthread_join(threads[i], NULL);

    printf("    %d pins held, %d refused\n", atomic_load(&pinned),
           atomic_load(&refused));

    CHECK(acquired == 3000);
    CHECK(atomic_load(&torn) == 0);
    /* Both happened, or the race the test is for was never run. */
    CHECK(atomic_load(&pinned) > 0);
    CHECK(atomic_load(&refused) > 0);
    CHECK(laser_pin(token) == NULL);

    return check_result();
}
