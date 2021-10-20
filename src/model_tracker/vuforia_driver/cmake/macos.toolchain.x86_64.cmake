# Standard settings

set (CMAKE_SYSTEM_NAME Darwin)
set (CMAKE_SYSTEM_VERSION 13)
set (UNIX True)
set (APPLE True)
set (MACOS True)
set (DRIVER_TEMPLATE_PLATFORM_APPLE 1)
set (DRIVER_TEMPLATE_PLATFORM_MACOS 1)

# Xcode installation to use for xcodebuild (export DEVELOPER_DIR)
exec_program(/usr/bin/xcode-select ARGS -print-path OUTPUT_VARIABLE CMAKE_XCODE_DEVELOPER_DIR)
set (CONST_DIR_XCODE_DEVELOPER "${CMAKE_XCODE_DEVELOPER_DIR}")

if(NOT EXISTS ${CONST_DIR_XCODE_DEVELOPER})
    message(FATAL_ERROR "Cmake can't find Xcode developer directory: \"${CONST_DIR_XCODE_DEVELOPER}\"")
endif()

set(ENV{DEVELOPER_DIR} "${CONST_DIR_XCODE_DEVELOPER}")

# We have to redefine CMAKE_MAKE_PROGRAM, because cmake --build uses
# the system's default xcode version and the build subprocess is outside of the DEVELOPER_DIR scope
set(CMAKE_MAKE_PROGRAM ${CONST_DIR_XCODE_DEVELOPER}/usr/bin/xcodebuild CACHE STRING "" FORCE)

message(STATUS "DEVELOPER_DIR=$ENV{DEVELOPER_DIR}")
message(STATUS "CMAKE_MAKE_PROGRAM ${CMAKE_MAKE_PROGRAM}")

set(MACOSSDKROOT_FOUND False)

if (NOT DEFINED CMAKE_MACOS_DEVELOPER_ROOT)
    file (GLOB _CMAKE_MACOS_SDKS "${CONST_DIR_XCODE_DEVELOPER}/Platforms/MacOSX.platform/Developer/SDKs/*")
    if (_CMAKE_MACOS_SDKS)
        list (SORT _CMAKE_MACOS_SDKS)
        list (REVERSE _CMAKE_MACOS_SDKS)
        list (GET _CMAKE_MACOS_SDKS 0 CMAKE_MACOS_DEVELOPER_ROOT)
    else (_CMAKE_MACOS_SDKS)
        # no SDK in the developer root dir
        return()
    endif (_CMAKE_MACOS_SDKS)

endif (NOT DEFINED CMAKE_MACOS_DEVELOPER_ROOT)

get_filename_component(MACOS_SDK_ROOT_FILE ${CMAKE_MACOS_DEVELOPER_ROOT} NAME CACHE)
string(REGEX MATCH "[0-9]+.[0-9]+" MACOSSDKROOT_VERSION "${MACOS_SDK_ROOT_FILE}")
set(MACOSSDKROOT_FOUND True)

mark_as_advanced(MACOS_SDK_ROOT_FILE)

if(NOT MACOSSDKROOT_FOUND)
    message(FATAL_ERROR "MacOS SDK Root is not found!")
endif()

set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE STRING "MacOS build architectures" FORCE)
set(TARGET_ARCH ${CMAKE_OSX_ARCHITECTURES} CACHE STRING "" FORCE)

# OPENAR_CMAKE_CXX_STANDARD property specifies the C++
# standard whose  features are requested to build this target.
# Supported values are 98, 11, 14 and 17.
set(OPENAR_CMAKE_CXX_STANDARD 17)

mark_as_advanced(CMAKE_OSX_DEPLOYMENT_TARGET)
mark_as_advanced(CMAKE_OSX_ARCHITECTURES)
mark_as_advanced(CMAKE_OSX_SYSROOT)
