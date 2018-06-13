cmake_minimum_required(VERSION 2.8.12)

project(m4lib)

add_definitions(
    -std=c++11
    -DWINDOWS
    -DCURL_STATICLIB
    -Werror
    -Wall
    -Wextra
)

if (PLATFORM STREQUAL "Android_x86")
    message(STATUS "Building for ST16")
    add_definitions(-D__androidx86__)
else()
    message(STATUS "Building for non-ST16")
endif()

if ( ZIGBEE STREQUAL "DISABLE_ZIGBEE")
     message(STATUS "Disable ZIGBEE")
     add_definitions(-DDISABLE_ZIGBEE)
 else()
     message(STATUS "Uising ZIGBEE")
 endif()

add_library(m4lib STATIC
    src/m4lib.cpp
    src/m4serial.cpp
    src/m4util.cpp
)

target_link_libraries(m4lib
    # nothing
)

