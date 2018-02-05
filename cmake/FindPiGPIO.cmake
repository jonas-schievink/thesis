# Includes
find_path(pigpio_INCLUDE_DIR
    NAMES pigpio.h
    HINTS /usr/local/include /usr/include)

# Libraries
find_library(pigpio_LIBRARY
    NAMES pigpio
    HINTS /usr/local/lib /usr/lib /lib)

# Set the plural form variables
set(pigpio_INCLUDE_DIRS ${pigpio_INCLUDE_DIR})
set(pigpio_INCLUDES     ${pigpio_INCLUDE_DIR})

# Handle REQUIRED, QUIET, and version arguments and set the <packagename>_FOUND variable.
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    pigpio DEFAULT_MSG
    pigpio_INCLUDE_DIR pigpio_LIBRARY)
