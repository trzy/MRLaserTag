# Driver Template
Implements a simple Vuforia Driver to feed camera frame and pose into Vuforia Engine.

# Building
Output binaries are placed in build/bin by default.

## Prerequisites
* Python

## Android
```
python build.py android
```
## UWP
```
python build.py uwp
```
## IOS
```
python build.py ios
```
## Set directory with Vuforia header
If you put this sample into the "samples" directory of your Vuforia SDK, the
Vuforia headers will be found automatically. If your SDK is located elsewhere
then you need to specify where. There are two ways to do this. For iOS, use the
-vf flag, and pass it the path to the directory that contains
Vuforia.framework. For other target platforms, use the -vh flag and pass it the
path to the SDK's "include" directory.

Ex: 
```
python build.py ios -vf '[vuforia-sdk-directory]/build'
python build.py android -vh '[vuforia-sdk-directory]/build/include'
```

# Usage
## Android
1. Add libRefDriverImpl.so from build/bin into your Android-app project.

    libRefDriverImpl.so can be added to an app by two ways depending on your build system: Using Android.mk or using Gradle. Please note that you need to substitute '[path-in-your-filesystem]' with the correct path.

    **Using Android.mk (e.g. ImageTargetsNative sample app)**

    Add libRefDriver-prebuilt definition to your Android.mk:
    ```
    include $(CLEAR_VARS)
    LOCAL_MODULE := libRefDriver-prebuilt
    LOCAL_SRC_FILES = [path-in-your-filesystem]/DriverTemplate/build/bin/Android/$(TARGET_ARCH_ABI)/libRefDriverImpl.so
    include $(PREBUILT_SHARED_LIBRARY)
    ```
    When defining your local module in the same Android.mk add libRefDriver-prebuilt as a dependency to your LOCAL_SHARED_LIBRARIES:
    ```
    LOCAL_SHARED_LIBRARIES := Vuforia-prebuilt libRefDriver-prebuilt
    ```

    **Using Gradle (e.g. VuforiaSamples sample app)**

    This can be done in your app/build.gradle with the following:
    ```
    android {
        sourceSets.main {
            jniLibs.srcDirs += '[path-in-your-filesystem]/DriverTemplate/build/bin/Android/'
        }
    }
    ```

2. Add following call to your source code before calling Vuforia::init();

    **Java**:
    ```
    Vuforia.setDriverLibrary("libRefDriverImpl.so");
    ```

    **C++**:
    ```
    Vuforia::setDriverLibrary("libRefDriverImpl.so", nullptr);
    ```

## UWP
1. Add DriverTemplate.dll from build/bin/uwp into your Visual Studio UWP-app project.
    - Import the DriverTemplate.dll to the root of your project. Remember to use a dll that matches your architecture (x86/x64) and build type (Debug/Release) configurations.
    - Click DriverTemplate.dll from the project file list and set property "Content" to True.

2. Add following call before calling Vuforia::init();
    ```
    Vuforia::setDriverLibrary("DriverTemplate.dll", nullptr);
    ```
