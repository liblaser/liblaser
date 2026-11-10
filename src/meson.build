# liblaser's own code, its symbol confinement, and what is installed for its
# consumers: liblaser.a and liblaser.pc.

# Everything this library promises, and nothing else: the union of laser.h and
# laser_disc.h. A function added to either and forgotten here does not fail to
# build, it fails to link in whoever calls it - confine.py checks both
# directions, but this is the one list to update.
laser_public_symbols = [
    'laser_set_log_cb',
    'laser_set_abort_cb',
    'laser_parse_token',
    'laser_acquire',
    'laser_release',
    'laser_token_not_ready',
    'laser_scsi_cdb',
    'laser_css_session_begin',
    'laser_css_session_end',
    'laser_read_blocks',
    'laser_read_blocks_ahead',
    'laser_read_cd_blocks',
    'laser_status_is_positional',
    'laser_region_mismatch',
    'laser_unreadable_blocks',
    'laser_refused_for_good',
    'laser_capacity',
    'laser_disc_identify',
    'laser_disc_identify_lang',
    'laser_media_poll',
    'laser_media_lock',
    'laser_media_eject',
    'laser_media_generation',
]

# Our own code, with warnings - the vendored targets are not ours to keep
# warning-clean. Default visibility: laser_pin() and laser_bot_send_locked()
# cross between objects and must survive as far as the partial link.
laser_warning_args = cc.get_supported_arguments(
    '-Wshadow', '-Wvla', '-Wswitch-enum')

# The public headers, and this directory for the internal ones.
laser_internal = static_library('laser-own',
    files(
        'registry.c',
        'usb.c',
        'bot.c',
        'scsi.c',
        'ready.c',
        'read.c',
        'medium.c',
        'disc.c',
    ),
    include_directories: [laser_public_inc, include_directories('.'),
                          libusb_inc, libudfread_inc],
    c_args: laser_warning_args,
    dependencies: threads_dep,
    override_options: ['warning_level=2'],
    install: false)

# SYMBOL CONFINEMENT. libbluray bundles its own libudfread, exporting the same
# udfread_* names, and an Android VLC links both archives - so which copy
# satisfies a reference would depend on archive member ordering. Hence nothing
# but laser_public_symbols leaves this archive: confine.py partially links the
# three internal archives into one relocatable object, demotes every other
# global to local, asserts the result in both directions, and archives it.
#
# The partial link is what makes this possible at all: localizing per object
# would break laser_pin(), called across objects, and libusb's files call
# into each other throughout.
liblaser = custom_target('liblaser',
    input: [laser_internal, libusb_internal, libudfread_internal],
    output: 'liblaser.a',
    command: [confine,
        '--output', '@OUTPUT@',
        '--symbols', ','.join(laser_public_symbols),
        '--cc'] + cc.cmd_array() + ['--', '@INPUT@'],
    install: true,
    install_dir: get_option('libdir'))

# -llog is declared here rather than left to whoever links this archive: our
# own registry.c and libusb's core.c (USE_SYSTEM_LOGGING_FACILITY in its
# Android config.h) both log through the Android logger. It is in Libs rather
# than only Libs.private because VLC asks pkg-config for --libs, not --static.
pkg = import('pkgconfig')
pkg.generate(
    name: 'liblaser',
    filebase: 'liblaser',
    description: 'Library for Accessing SCSI External Readers - SCSI-MMC over USB to optical drives',
    version: meson.project_version(),
    libraries: ['-L${libdir}', '-llaser', '-llog'],
    libraries_private: ['-llog'])

# For use as a meson subproject: the public headers only.
liblaser_dep = declare_dependency(
    link_with: liblaser,
    include_directories: laser_public_inc)
