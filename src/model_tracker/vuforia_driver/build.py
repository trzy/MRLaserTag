#===============================================================================
# Copyright (c) 2020 PTC Inc. All Rights Reserved.
#
# Confidential and Proprietary - Protected under copyright and other laws.
# Vuforia is a trademark of PTC Inc., registered in the United States and other
# countries.
#===============================================================================
import argparse
import os
import shutil
import subprocess
from distutils.spawn import find_executable 

PLATFORM_ANDROID = "android"
PLATFORM_UWP = "uwp"
PLATFORM_IOS = "ios"
PLATFORM_MACOS = "macos"

VALID_ARCHS = {
    PLATFORM_ANDROID: ["armeabi-v7a", "arm64-v8a"],
    PLATFORM_UWP: ["x86", "x64", "ARM", "ARM64"],
    PLATFORM_IOS: ["arm64", "x86_64"],
    PLATFORM_MACOS: ["x86_64"]
}

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("platform",
                        help="Platform: {0} or {1} or {2} or {3}".format(PLATFORM_ANDROID, PLATFORM_UWP, PLATFORM_IOS, PLATFORM_MACOS),
                        choices=[PLATFORM_ANDROID, PLATFORM_UWP, PLATFORM_IOS, PLATFORM_MACOS])

    parser.add_argument("-a", "--arch",
                        help="Architectures. Comma separated list values. Values for Android: armeabi-v7a, arm64-v8a, x86. Values for UWP: x86, x64, ARM, ARM64. Values for iOS: arm64, x86_64. Values for MacOS: x86_64")

    parser.add_argument("-i", "--install",
                        help="Install directory.",
                        default=os.path.join("build"))

    parser.add_argument("-o", "--output",
                        help="Location of generated build files.",
                        default=os.path.join("build"))

    parser.add_argument("-bt", "--build-type",
                        help="Build type. Release or Debug",
                        choices=["Release", "Debug", "RelWithDebInfo", "MinSizeRel"],
                        default="Release")

    parser.add_argument("-vh", "--vuforia-header-dir",
                        help="Directory that contains Vuforia/Driver/Driver.h.",
                        default=None)

    parser.add_argument("-vf", "--vuforia-framework-dir",
                        help="Directory that contains Vuforia.framework (iOS only).",
                        default=os.path.join("..", "..", "build"))

    parser.add_argument("-c", "--clean",
                        help="Delete all generated and built outputs and exit.",
                        action='store_true')

    args = parser.parse_args()
    platform = args.platform

    arch_string = args.arch
    if arch_string:
        archs = arch_string.split(",")
    else:
        archs = VALID_ARCHS[platform]
        print("No architectures selected. using defaults: {0}".format(str(archs)))

    for arch in archs:
        if arch not in VALID_ARCHS[platform]:
            print("error: Invalid architecture: {0}".format(arch))
            exit(1)

    cwd = os.getcwd()
    output_rootdir = os.path.abspath(args.output)
    install_rootdir = os.path.abspath(args.install)
    build_type = args.build_type
    toolchain_dir = os.path.join(cwd, "cmake")

    # For iOS, use the framework path unless the header path was explicitly specified.
    # For other platforms, use the header path.
    if platform == PLATFORM_IOS:
        if args.vuforia_header_dir:
            vuforia_header_dir = os.path.abspath(args.vuforia_header_dir)
            vuforia_framework_dir = None
        else:
            vuforia_header_dir = None
            vuforia_framework_dir = os.path.abspath(args.vuforia_framework_dir)
    else:
        vuforia_framework_dir = None
        # Construct the default header dir if it was unspecified
        if args.vuforia_header_dir:
            vuforia_header_dir = os.path.abspath(args.vuforia_header_dir)
        else:
            vuforia_header_dir = os.path.abspath(os.path.join("..", "..", "build", "include"))
        
    # If we're on windows fix up paths to remove backslashes that break cmake install
    if os.name == 'nt':
        install_rootdir = install_rootdir.replace('\\', '/')
        vuforia_header_dir = vuforia_header_dir.replace('\\', '/')

    if args.clean:
        if os.path.exists(output_rootdir):
            print("Cleaning outputs...")
            output_bin_dir = os.path.join(output_rootdir, "bin")
            if platform == PLATFORM_ANDROID:
                shutil.rmtree(os.path.join(output_rootdir, "android"), ignore_errors=True)
                shutil.rmtree(os.path.join(output_bin_dir, "android"), ignore_errors=True)
            elif platform == PLATFORM_UWP:
                shutil.rmtree(os.path.join(output_rootdir, "uwp"), ignore_errors=True)
                shutil.rmtree(os.path.join(output_bin_dir, "WindowsStore"), ignore_errors=True)
            elif platform == PLATFORM_IOS:
                shutil.rmtree(os.path.join(output_rootdir, "ios"), ignore_errors=True)
                shutil.rmtree(os.path.join(output_bin_dir, "Debug-iphoneos"), ignore_errors=True)
                shutil.rmtree(os.path.join(output_bin_dir, "Release-iphoneos"), ignore_errors=True)
            elif platform == PLATFORM_MACOS:
                shutil.rmtree(os.path.join(output_rootdir, "macos"), ignore_errors=True)
                shutil.rmtree(os.path.join(output_bin_dir, "Darwin"), ignore_errors=True)
            # Remove bin directory if empty
            if os.path.isdir(output_bin_dir) and not os.listdir(output_bin_dir):
                os.rmdir(output_bin_dir)
            # Remove output directory if empty
            if os.path.isdir(output_rootdir) and not os.listdir(output_rootdir):
                os.rmdir(output_rootdir)
            print("done.")
        else:
            print("Already clean.")
        exit(0)

    if not os.path.exists(output_rootdir):
        os.makedirs(output_rootdir)
        
    cmakePath = find_executable("cmake")
    # On iOS we need a full path for XCode generator.
    # If we have not found it (which can happen), then we just use the full path.
    # On Windows and Android we throw an error, if cmake is not installed.
    if platform == PLATFORM_IOS and not cmakePath:
        cmakePath = "/Applications/CMake.app/Contents/bin/cmake"
    if platform == PLATFORM_MACOS and not cmakePath:
        cmakePath = "/Applications/CMake.app/Contents/bin/cmake"
    if not cmakePath:
        print("error: Please install cmake and make sure that it is set in the PATH")
    else:
        print("Using cmake: {0}".format(cmakePath))

    # Check that ninja is installed for Android build.
    if platform == PLATFORM_ANDROID:
        ninjaPath = find_executable("ninja")
        if not ninjaPath:
            print("error: Please install Ninja (https://ninja-build.org/) and make sure that it is set in the PATH")
        else:
            print("Using ninja: {0}".format(ninjaPath))

    for arch in archs:
        print("Generating arch: {0}".format(arch))

        if platform == PLATFORM_ANDROID:
            toolchain_file = os.path.join(toolchain_dir, "android.toolchain.{0}.cmake".format(arch))
            generator = "Ninja"
            generator_platform = None
        elif platform == PLATFORM_UWP:
            toolchain_file = os.path.join(toolchain_dir, "uwp.toolchain.{0}.cmake".format(arch))
            if arch == "x86":
                generator = "Visual Studio 16 2019"
                generator_platform = "Win32"
            elif arch == "x64":
                generator = "Visual Studio 16 2019"
                generator_platform = "x64"
            elif arch == "ARM":
                generator = "Visual Studio 16 2019"
                generator_platform = "ARM"
            elif arch == "ARM64":
                generator = "Visual Studio 16 2019"
                generator_platform = "ARM64"
        elif platform == PLATFORM_IOS:
            toolchain_file = os.path.join(toolchain_dir, "ios.toolchain.{0}.cmake".format(arch))
            generator = "Xcode"
            generator_platform = None
        elif platform == PLATFORM_MACOS:
            toolchain_file = os.path.join(toolchain_dir, "macos.toolchain.{0}.cmake".format(arch))
            generator = "Xcode"
            generator_platform = None

        output_dir = os.path.join(output_rootdir, platform, arch)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        cmake_command = [cmakePath]
        cmake_command.extend(["-G", generator])
        if generator_platform:
            cmake_command.append("-A")
            cmake_command.append(generator_platform)
        if vuforia_header_dir:
            cmake_command.extend([
                "-DVUFORIA_HEADER_DIR='{0}'".format(vuforia_header_dir)])
        elif vuforia_framework_dir:
             cmake_command.extend([
                "-DVUFORIA_FRAMEWORK_DIR='{0}'".format(vuforia_framework_dir)])
        cmake_command.extend([
            "-DCMAKE_INSTALL_PREFIX='{0}'".format(install_rootdir),
            "-DCMAKE_TOOLCHAIN_FILE='{0}'".format(toolchain_file),
            "-DCMAKE_BUILD_TYPE={0}".format(build_type),
            cwd])

        ret = subprocess.call(cmake_command, cwd=output_dir)

        if ret != 0:
            print("error: Project generation with cmake failed.")
            exit(1)

        print("Building arch: {0}".format(arch))

        ret = subprocess.call(
            [cmakePath,
            "--build", ".",
            "--target", "install",
            "--config", build_type],
            cwd=output_dir)

        if ret != 0:
            print("error: Project generation with cmake failed.")
            exit(1)

    print("Project generation and compilation succeeeded.")
    exit(0)
