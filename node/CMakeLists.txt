project(PiBot)
cmake_minimum_required(VERSION 2.8)

# Search for CMake modules in `cmake/` directory
set(CMAKE_MODULE_PATH ${PiBot_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH})

# Add required dependencies
find_package(PiGPIO REQUIRED)

# add a target to generate API documentation with Doxygen
# Source: https://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/
find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)
# you can now run `make doc` to generate API documentation


add_library(pigpio++ src/PiGPIO.cpp)
target_link_libraries(pigpio++ ${pigpio_LIBRARY})

add_executable(pibot src/GPIOEncoder.cpp src/Motor.cpp src/PiBot.cpp)
target_link_libraries(pibot pigpio++ pthread)

# Note: This isn't the correct way to include `pthread`, but it works on Linux
