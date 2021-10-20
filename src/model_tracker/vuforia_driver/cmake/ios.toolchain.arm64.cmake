set(IOS_PLATFORM OS)
set(IOS_PLATFORM_NAME "iphoneos")
set(IOS_PLATFORM_LOCATION "iPhoneOS.platform")
set(IOS_ARCH "arm64")

# In order to sign DriverTemplate framework make sure to set the following values for:
#   * Code sign identity
#   * Development Team ID
#
# set(CMAKE_XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "" CACHE STRING "Code sign identity")
# set(CMAKE_XCODE_ATTRIBUTE_DEVELOPMENT_TEAM "" CACHE STRING "Development Team ID")

include(${CMAKE_CURRENT_LIST_DIR}/ios.toolchain.cmake)
