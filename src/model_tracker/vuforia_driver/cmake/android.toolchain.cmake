set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_ANDROID_NDK_TOOLCHAIN_VERSION clang)
set(DRIVER_TEMPLATE_PLATFORM_ANDROID 1)

set(ANDROID_NATIVE_API_LEVEL 22)
set(ANDROID_STL c++_static)

find_path(ANDROID_NDK_TOOLCHAIN_PATH
            NAMES android.toolchain.cmake
            PATHS
            $ENV{ANDROID_NDK_HOME}/build/cmake)

if(NOT ANDROID_NDK_TOOLCHAIN_PATH)
    message(FATAL_ERROR "Android NDK not found. Define ANDROID_NDK_HOME env variable and point it to Android NDK root directory.")
endif()

include(${ANDROID_NDK_TOOLCHAIN_PATH}/android.toolchain.cmake)
