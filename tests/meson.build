# liblaser's tests: its own code against simulated devices, with no drive and
# no USB. Built with -Dtests=true, in a build for the machine they will run
# on, and run with "meson test".
#
# "meson test" builds everything first, the vendored libusb included, which
# is configured for Android. Where that does not build, the tests alone do:
#
#   meson compile -C <builddir> test_transport test_read test_registry \
#                               test_udf_window test_identify
#   meson test -C <builddir> --no-rebuild
#
# Each program includes the source files it tests, so as to reach what they
# keep to themselves, and supplies the functions those files call in the
# layer below - libusb, the command layer, libudfread - as a simulation of
# it. None links liblaser or libusb, and one only links libudfread: of the
# rest, the headers alone are used.
#
#   test_transport   - bot.c, scsi.c and medium.c against a scripted
#                      Bulk-Only device
#   test_read        - read.c against a command layer failing chosen blocks
#   test_registry    - registry.c, operations in flight against teardowns
#   test_udf_window  - the read window disc.c gives libudfread
#   test_identify    - disc.c, through the vendored libudfread, identifying
#                      the disc images of tests/images: see make_images.py
#                      there, which wrote them and says what each holds.
#                      libudfread reports on stderr the volumes it does not
#                      find on the images that have none: that is expected

laser_test_inc = [laser_public_inc, include_directories('../src'),
                  libusb_inc, libudfread_inc]

# The included sources have static functions a given test does not call.
laser_test_args = laser_warning_args + cc.get_supported_arguments(
    '-Wno-unused-function')

foreach name : ['test_transport', 'test_read', 'test_registry',
                'test_udf_window']
    exe = executable(name, name + '.c',
        include_directories: laser_test_inc,
        c_args: laser_test_args,
        dependencies: threads_dep,
        override_options: ['warning_level=2'],
        install: false)
    test(name, exe, timeout: 120)
endforeach

test_identify = executable('test_identify', 'test_identify.c',
    include_directories: laser_test_inc,
    c_args: laser_test_args,
    link_with: libudfread_internal,
    dependencies: threads_dep,
    override_options: ['warning_level=2'],
    install: false)
test('test_identify', test_identify,
    args: [meson.current_source_dir() / 'images'],
    timeout: 120)
