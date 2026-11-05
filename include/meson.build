# liblaser's public headers: laser.h, laser_disc.h, and laser_version.h
# generated here. Installed directly in the include directory, not in a
# subdirectory: the consumers - VLC's laser and cdrom.c, libdvdcss - include
# <laser.h> through the contrib prefix's include path, not through
# liblaser.pc's Cflags.

# laser_version.h, from the project version - set in the top meson.build, the
# one place it is set.
laser_version = meson.project_version().split('.')
laser_version_conf = configuration_data()
laser_version_conf.set('LASER_VERSION_MAJOR', laser_version[0])
laser_version_conf.set('LASER_VERSION_MINOR', laser_version[1])
laser_version_conf.set('LASER_VERSION_MICRO', laser_version[2])

configure_file(
    input: 'laser_version.h.in',
    output: 'laser_version.h',
    configuration: laser_version_conf,
    install: true,
    install_dir: get_option('includedir'))

install_headers('laser.h', 'laser_disc.h')

# This directory and its counterpart in the build directory, where
# laser_version.h is generated: everything a consumer may include, and nothing
# else.
laser_public_inc = include_directories('.')
