# Standard settings
set (CMAKE_SYSTEM_NAME Darwin)
set (CMAKE_SYSTEM_VERSION 13)
set (UNIX True)
set (APPLE True)
set (IOS True)
set (DRIVER_TEMPLATE_PLATFORM_APPLE 1)
set (DRIVER_TEMPLATE_PLATFORM_IOS 1)

set (CMAKE_OSX_DEPLOYMENT_TARGET "12.0" CACHE STRING "Force unset of the deployment target for iOS" FORCE)
set (CMAKE_XCODE_ATTRIBUTE_IPHONEOS_DEPLOYMENT_TARGET "12.0" CACHE STRING "Force unset of the deployment target for iOS" FORCE)

# Skip the platform compiler checks for cross compiling
set (CMAKE_CXX_COMPILER_WORKS TRUE)
set (CMAKE_C_COMPILER_WORKS TRUE)

# All iOS/Darwin specific settings - some may be redundant
set (CMAKE_SHARED_LIBRARY_PREFIX "lib")
set (CMAKE_SHARED_LIBRARY_SUFFIX ".dylib")
set (CMAKE_SHARED_MODULE_PREFIX "lib")
set (CMAKE_SHARED_MODULE_SUFFIX ".so")
set (CMAKE_MODULE_EXISTS 1)
set (CMAKE_DL_LIBS "")

set (CMAKE_C_OSX_COMPATIBILITY_VERSION_FLAG "-compatibility_version ")
set (CMAKE_C_OSX_CURRENT_VERSION_FLAG "-current_version ")
set (CMAKE_CXX_OSX_COMPATIBILITY_VERSION_FLAG "${CMAKE_C_OSX_COMPATIBILITY_VERSION_FLAG}")
set (CMAKE_CXX_OSX_CURRENT_VERSION_FLAG "${CMAKE_C_OSX_CURRENT_VERSION_FLAG}")

# Hidden visibilty is required for cxx on iOS
set (CMAKE_C_FLAGS_INIT "")
set (CMAKE_CXX_FLAGS_INIT "-fvisibility=hidden -fvisibility-inlines-hidden")

set (CMAKE_C_LINK_FLAGS "-Wl,-search_paths_first ${CMAKE_C_LINK_FLAGS}")
set (CMAKE_CXX_LINK_FLAGS "-Wl,-search_paths_first ${CMAKE_CXX_LINK_FLAGS}")

set (CMAKE_PLATFORM_HAS_INSTALLNAME 1)
set (CMAKE_SHARED_LIBRARY_CREATE_C_FLAGS "-dynamiclib -headerpad_max_install_names")
set (CMAKE_SHARED_MODULE_CREATE_C_FLAGS "-bundle -headerpad_max_install_names")
set (CMAKE_SHARED_MODULE_LOADER_C_FLAG "-Wl,-bundle_loader,")
set (CMAKE_SHARED_MODULE_LOADER_CXX_FLAG "-Wl,-bundle_loader,")
set (CMAKE_FIND_LIBRARY_SUFFIXES ".dylib" ".so" ".a")

# hack: if a new cmake (which uses CMAKE_INSTALL_NAME_TOOL) runs on an old build tree
# (where install_name_tool was hardcoded) and where CMAKE_INSTALL_NAME_TOOL isn't in the cache
# and still cmake didn't fail in CMakeFindBinUtils.cmake (because it isn't rerun)
# hardcode CMAKE_INSTALL_NAME_TOOL here to install_name_tool, so it behaves as it did before, Alex
if (NOT DEFINED CMAKE_INSTALL_NAME_TOOL)
	find_program(CMAKE_INSTALL_NAME_TOOL install_name_tool)
endif (NOT DEFINED CMAKE_INSTALL_NAME_TOOL)

message(STATUS "IOS_PLATFORM=${IOS_PLATFORM}")
	
set(TARGET_ARCH "${IOS_ARCH}")
set(CMAKE_XCODE_EFFECTIVE_PLATFORMS "-${IOS_PLATFORM_NAME}")

# Setup iOS developer location unless specified manually with CMAKE_IOS_DEVELOPER_ROOT
exec_program(/usr/bin/xcode-select ARGS -print-path OUTPUT_VARIABLE CMAKE_XCODE_DEVELOPER_DIR)
set (XCODE_ROOT "${CMAKE_XCODE_DEVELOPER_DIR}/Platforms/${IOS_PLATFORM_LOCATION}/Developer")
if (NOT DEFINED CMAKE_IOS_DEVELOPER_ROOT)
	if (EXISTS ${XCODE_ROOT})
		set (CMAKE_IOS_DEVELOPER_ROOT ${XCODE_ROOT})
	else ()
		message(FATAL_ERROR "Xcode Developer directory not found!")
	endif ()
endif (NOT DEFINED CMAKE_IOS_DEVELOPER_ROOT)
set (CMAKE_IOS_DEVELOPER_ROOT ${CMAKE_IOS_DEVELOPER_ROOT} CACHE PATH "Location of iOS Platform")

message(STATUS "CMAKE_IOS_DEVELOPER_ROOT: ${CMAKE_IOS_DEVELOPER_ROOT}")

if (NOT DEFINED CMAKE_IOS_SDK_ROOT)
    file (GLOB _CMAKE_IOS_SDKS "${CMAKE_IOS_DEVELOPER_ROOT}/SDKs/*")
    if (_CMAKE_IOS_SDKS)
        list (SORT _CMAKE_IOS_SDKS)
        list (REVERSE _CMAKE_IOS_SDKS)
        list (GET _CMAKE_IOS_SDKS 0 CMAKE_IOS_SDK_ROOT)
    else (_CMAKE_IOS_SDKS)
        # no SDK in the developer root dir
        return()
    endif (_CMAKE_IOS_SDKS)

endif (NOT DEFINED CMAKE_IOS_SDK_ROOT)

set(IOSSDKROOT_FOUND True)
get_filename_component(IOS_SDK_ROOT_FILE ${CMAKE_IOS_SDK_ROOT} NAME CACHE)
string(REGEX MATCH "[0-9]+.[0-9]+" IOSSDKROOT_VERSION "${IOS_SDK_ROOT_FILE}")

mark_as_advanced(IOS_SDK_ROOT_FILE)

set(CMAKE_IOS_SDK_ROOT 	${CMAKE_IOS_SDK_ROOT} CACHE PATH  "Location of the selected iOS SDK") 

# Set the sysroot default to the most recent SDK
set (CMAKE_OSX_SYSROOT ${CMAKE_IOS_SDK_ROOT} CACHE PATH "Sysroot used for iOS support")

set (CMAKE_OSX_ARCHITECTURES ${IOS_ARCH} CACHE string  "Build architecture for iOS")

# Set the find root to the iOS developer roots and to user defined paths
set (CMAKE_FIND_ROOT_PATH ${CMAKE_IOS_DEVELOPER_ROOT} ${CMAKE_IOS_SDK_ROOT} ${CMAKE_PREFIX_PATH} CACHE string  "iOS find search path root")

# default to searching for frameworks first
set (CMAKE_FIND_FRAMEWORK FIRST)

# set up the default search directories for frameworks
set (CMAKE_SYSTEM_FRAMEWORK_PATH
	${CMAKE_IOS_SDK_ROOT}/System/Library/Frameworks
	${CMAKE_IOS_SDK_ROOT}/System/Library/PrivateFrameworks
	${CMAKE_IOS_SDK_ROOT}/Developer/Library/Frameworks
)

# only search the iOS sdks, not the remainder of the host filesystem
set (CMAKE_FIND_ROOT_PATH_MODE_PROGRAM ONLY)
set (CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set (CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)


mark_as_advanced(CMAKE_OSX_DEPLOYMENT_TARGET)
mark_as_advanced(CMAKE_OSX_ARCHITECTURES)
mark_as_advanced(CMAKE_FIND_ROOT_PATH)
mark_as_advanced(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM)
mark_as_advanced(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY)
mark_as_advanced(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE)
mark_as_advanced(CMAKE_OSX_SYSROOT)
mark_as_advanced(CMAKE_IOS_SDK_ROOT)
mark_as_advanced(CMAKE_IOS_DEVELOPER_ROOT)
mark_as_advanced(IOS_PLATFORM)


