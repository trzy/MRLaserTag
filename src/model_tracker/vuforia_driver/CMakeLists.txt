#===============================================================================
#Copyright (c) 2020 PTC Inc. All Rights Reserved.
#
#Vuforia is a trademark of PTC Inc., registered in the United States and other
#countries.
#==============================================================================
cmake_minimum_required(VERSION 3.9.0)

project(DriverTemplate)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang|AppleClang|Apple")
    set(CLANG_COMPILER 1)
elseif(MSVC)
    set(MSVC_COMPILER 1)
else()
    message(FATAL_ERROR "The Compiler ${CMAKE_CXX_COMPILER_ID} is not supported")
endif()

if (NOT VUFORIA_HEADER_DIR AND NOT VUFORIA_FRAMEWORK_DIR)
    message(FATAL_ERROR "Either VUFORIA_HEADER_DIR or VUFORIA_FRAMEWORK_DIR must be defined. Define by -DVUFORIA_HEADER_DIR='[dir]'")
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL "Android")
    if(NOT DEFINED ENV{ANDROID_HOME})
        message( FATAL_ERROR "Error: ANDROID_HOME is not defined." )
    endif()
endif()

set(DRIVER_DIR_SRC                 src)
set(DRIVER_DIR_INC                 include)
set(DRIVER_TEMPLATE_TARGET_NAME             "DriverTemplate")

set(DRIVER_TEMPLATE_SRC
    ${DRIVER_DIR_SRC}/RefDriverImpl.cpp
    ${DRIVER_DIR_SRC}/RefImplSequencePlayer.cpp)

set(DRIVER_TEMPLATE_INC
    ${DRIVER_DIR_INC}/RefDriverImpl.h
    ${DRIVER_DIR_INC}/RefDriverImplData.h
    ${DRIVER_DIR_INC}/RefImplSequencePlayer.h
    ${DRIVER_DIR_INC}/Platform.h)

if(DRIVER_TEMPLATE_PLATFORM_ANDROID)
    set(DRIVER_TEMPLATE_PLATFORM_SRC
        ${DRIVER_DIR_SRC}/Platform_Android.cpp)
elseif(DRIVER_TEMPLATE_PLATFORM_APPLE)
    set(DRIVER_TEMPLATE_PLATFORM_SRC
        ${DRIVER_DIR_SRC}/Platform_Apple.mm)
elseif(DRIVER_TEMPLATE_PLATFORM_UWP)
    set(DRIVER_TEMPLATE_PLATFORM_SRC
        ${DRIVER_DIR_SRC}/Platform_UWP.cpp)
endif()

add_library(${DRIVER_TEMPLATE_TARGET_NAME}
            SHARED
            ${DRIVER_TEMPLATE_SRC}
            ${DRIVER_TEMPLATE_PLATFORM_SRC}
            ${DRIVER_TEMPLATE_INC})

target_include_directories(${DRIVER_TEMPLATE_TARGET_NAME}
        PRIVATE
        ${DRIVER_DIR_INC})

if(VUFORIA_HEADER_DIR)
    target_include_directories(${DRIVER_TEMPLATE_TARGET_NAME}
        PRIVATE
        ${VUFORIA_HEADER_DIR})
endif()

if(DRIVER_TEMPLATE_PLATFORM_IOS)
    if(VUFORIA_FRAMEWORK_DIR)
        target_compile_options(${DRIVER_TEMPLATE_TARGET_NAME} PRIVATE -F ${VUFORIA_FRAMEWORK_DIR})
    endif()

    set_target_properties(${DRIVER_TEMPLATE_TARGET_NAME} PROPERTIES
         FRAMEWORK TRUE
         PUBLIC_HEADER dynamicFramework.h
         )

    set_target_properties(${DRIVER_TEMPLATE_TARGET_NAME} PROPERTIES
         MACOSX_FRAMEWORK_INFO_PLIST  "${CMAKE_CURRENT_SOURCE_DIR}/Info.plist")

    # Set up the dynamic load path properly for an embeddable framework
    set_target_properties(${DRIVER_TEMPLATE_TARGET_NAME} PROPERTIES XCODE_ATTRIBUTE_DYLIB_INSTALL_NAME_BASE "@rpath")
    set_target_properties(${DRIVER_TEMPLATE_TARGET_NAME} PROPERTIES XCODE_ATTRIBUTE_LD_DYLIB_INSTALL_NAME "$(DYLIB_INSTALL_NAME_BASE:standardizepath)/$(EXECUTABLE_PATH)")
    set_target_properties(${DRIVER_TEMPLATE_TARGET_NAME} PROPERTIES XCODE_ATTRIBUTE_LD_RUNPATH_SEARCH_PATHS "@executable_path/Frameworks @loader_path/Frameworks")

    target_link_libraries(${DRIVER_TEMPLATE_TARGET_NAME}
        "-framework Foundation"
        "-framework CoreFoundation"
        "-framework UIKit"
        "-framework CoreGraphics"
        "-framework QuartzCore"
        )

endif()


# Enable warnings as errors and optimizations for Release builds.
if(CLANG_COMPILER)
     target_compile_options(${DRIVER_TEMPLATE_TARGET_NAME} PRIVATE -Wall -Werror)
     target_compile_options(${DRIVER_TEMPLATE_TARGET_NAME} PRIVATE $<$<CONFIG:Release>:-O3>)
elseif(MSVC_COMPILER)
     target_compile_options(${DRIVER_TEMPLATE_TARGET_NAME} PRIVATE /WX /W4)
     target_compile_options(${DRIVER_TEMPLATE_TARGET_NAME} PRIVATE $<$<CONFIG:Release>:/Ox>)
endif()

# Link with Android log library.
if(DRIVER_TEMPLATE_PLATFORM_ANDROID)
    target_link_libraries(${DRIVER_TEMPLATE_TARGET_NAME}
        log)
endif()

if(DRIVER_TEMPLATE_PLATFORM_UWP)
    target_compile_options(${DRIVER_TEMPLATE_TARGET_NAME} PRIVATE /ZW)
endif()

set (DRIVER_TEMPLATE_INSTALL_DIR ${CMAKE_SYSTEM_NAME}/${CMAKE_BUILD_TYPE})
if(DRIVER_TEMPLATE_PLATFORM_IOS)
    set (DRIVER_TEMPLATE_INSTALL_DIR $<CONFIG>-iphoneos)
endif()
  
install(TARGETS ${DRIVER_TEMPLATE_TARGET_NAME}
    RUNTIME DESTINATION bin/${DRIVER_TEMPLATE_INSTALL_DIR}/${TARGET_ARCH}
    LIBRARY DESTINATION bin/${DRIVER_TEMPLATE_INSTALL_DIR}/${TARGET_ARCH}
    ARCHIVE DESTINATION bin/${DRIVER_TEMPLATE_INSTALL_DIR}/${TARGET_ARCH}
    FRAMEWORK DESTINATION bin/${DRIVER_TEMPLATE_INSTALL_DIR}/${TARGET_ARCH})

