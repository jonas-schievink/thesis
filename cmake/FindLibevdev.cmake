# Use pkg-config to find libevdev
find_package(PkgConfig)
pkg_check_modules(PC_LIBEVDEV QUIET libevdev)

FIND_PATH(
    LIBEVDEV_INCLUDE_DIR libevdev/libevdev.h
    HINTS ${PC_LIBEVDEV_INCLUDEDIR} ${PC_LIBEVDEV_INCLUDE_DIRS}
    /usr/include
    /usr/local/include
    ${LIBEVDEV_PATH_INCLUDES}
)

FIND_LIBRARY(
    LIBEVDEV_LIBRARY
    NAMES evdev libevdev
    HINTS ${PC_LIBEVDEV_LIBDIR} ${PC_LIBEVDEV_LIBRARY_DIRS}
    PATHS ${ADDITIONAL_LIBRARY_PATHS}
        ${LIBEVDEV_PATH_LIB}
)

# Set plural form variables
set(LIBEVDEV_LIBRARIES ${LIBEVDEV_LIBRARY})
set(LIBEVDEV_INCLUDE_DIRS ${LIBEVDEV_INCLUDE_DIR})

# Handle find_package arguments like QUIET and REQUIRED
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libevdev  DEFAULT_MSG
                                  LIBEVDEV_LIBRARY LIBEVDEV_INCLUDE_DIR)
