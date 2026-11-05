# liblaser - Library for Accessing SCSI External Readers.
#
# Builds liblaser.a, a static archive with a single member exporting exactly
# the public API, and installs it with laser.h, laser_disc.h, laser_version.h
# and liblaser.pc.
#
# Layout:
#   include/            - the public headers; include/meson.build
#   src/                - registry.c / usb.c / bot.c / disc.c, the SCSI layer
#                         scsi.c / ready.c / read.c / medium.c, and the
#                         internal headers; src/meson.build
#   contrib/libusb/     - vendored libusb 1.0.30, a complete upstream release
#                         tree. Compiled here rather than through its own
#                         configure; its Android config.h is the one upstream
#                         ships at contrib/libusb/android/config.h for its own
#                         ndk-build, which is why the whole tree is vendored.
#   contrib/libudfread/ - vendored libudfread, used by src/disc.c. udfread.c
#                         only includes config.h behind HAVE_CONFIG_H, which is
#                         never defined here.
#   tools/confine.py    - the symbol confinement step, see src/meson.build.
#
# The vendored trees are upstream's as released, so they are described here
# rather than given a meson.build of their own.

project('liblaser', 'c',
    version: '0.3.1',
    license: 'LGPL-2.1-or-later',
    meson_version: '>= 0.60.0',
    # GNU C11: _Static_assert in bot.c, and ##__VA_ARGS__ in LOG_QUIRK().
    default_options: ['c_std=gnu11'])

cc = meson.get_compiler('c')
threads_dep = dependency('threads')

libusb_inc = include_directories('contrib/libusb/libusb')
libudfread_inc = include_directories('contrib/libudfread/src')

# Vendored libusb, Linux/POSIX backend only. This is upstream's Android list
# (android/jni/libusb.mk). linux_netlink.c is not optional even though nothing
# here enumerates: linux_usbfs.c references linux_netlink_start_event_monitor()
# behind a runtime test, so leaving it out is an undefined symbol at link time.
#
# The config directory is on this target's include path only: it holds a file
# called config.h, and only libusb asks for one.
libusb_internal = static_library('laser-libusb',
    files(
        'contrib/libusb/libusb/core.c',
        'contrib/libusb/libusb/descriptor.c',
        'contrib/libusb/libusb/hotplug.c',
        'contrib/libusb/libusb/io.c',
        'contrib/libusb/libusb/strerror.c',
        'contrib/libusb/libusb/sync.c',
        'contrib/libusb/libusb/os/linux_usbfs.c',
        'contrib/libusb/libusb/os/linux_netlink.c',
        'contrib/libusb/libusb/os/events_posix.c',
        'contrib/libusb/libusb/os/threads_posix.c',
    ),
    include_directories: [
        include_directories('contrib/libusb/android'),
        include_directories('contrib/libusb/libusb/os'),
        libusb_inc,
    ],
    dependencies: threads_dep,
    gnu_symbol_visibility: 'hidden',
    override_options: ['warning_level=0'],
    install: false)

# Vendored libudfread. Not default_blockinput.c (src/disc.c supplies its own
# SCSI-backed block input) and not udfread-version.c (only referenced when
# HAVE_UDFREAD_VERSION_H is defined, by libudfread's own build).
libudfread_internal = static_library('laser-udfread',
    files(
        'contrib/libudfread/src/udfread.c',
        'contrib/libudfread/src/ecma167.c',
    ),
    include_directories: libudfread_inc,
    gnu_symbol_visibility: 'hidden',
    override_options: ['warning_level=0'],
    install: false)

# Found here, where tools/ is: src/meson.build runs it.
confine = find_program('tools/confine.py')

# The public headers first: src/ compiles against them.
subdir('include')
subdir('src')
