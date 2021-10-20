/*===============================================================================
Copyright (c) 2020 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/
#include "RefDriverImpl.h"
#include "RefDriverImplData.h"
#include "RefImplSequencePlayer.h"
#include "Platform.h"
#include "Format.h"

#include <stdlib.h>
#include <stdio.h>

#include <windows.h>
#include <locale>
#include <codecvt>

/*-------------------------------------------------------------------------------------------------------------------*/
// C APIs to init and deinit the Vuforia driver Impl.
/*-------------------------------------------------------------------------------------------------------------------*/
namespace
{
    RefDriverImpl* g_RefDriverImplInstance = nullptr;
}
extern "C"
{
    Vuforia::Driver::VuforiaDriver* vuforiaDriver_init(Vuforia::Driver::PlatformData* platformData, void* userdata)
    {
        if (g_RefDriverImplInstance == nullptr)
        {
            g_RefDriverImplInstance = new RefDriverImpl(platformData, userdata);
            return g_RefDriverImplInstance;
        }
        // Attempting to init multiple instances is considered an error
        return nullptr;
    }
    void vuforiaDriver_deinit(Vuforia::Driver::VuforiaDriver* instance)
    {
        if (instance == g_RefDriverImplInstance)
        {
            delete static_cast<RefDriverImpl*>(instance);
            g_RefDriverImplInstance = nullptr;
        }
    }
    uint32_t vuforiaDriver_getAPIVersion()
    {
        return Vuforia::Driver::VUFORIA_DRIVER_API_VERSION;
    }
    uint32_t vuforiaDriver_getLibraryVersion(char* outString, const uint32_t maxLength)
    {
        std::string versionCode = "RefDriverImpl-v1";
        uint32_t numBytes = static_cast<uint32_t>(versionCode.size() > maxLength ? maxLength : versionCode.size());
        memcpy(outString, versionCode.c_str(), numBytes);
        return numBytes;
    }
} // extern "C"

static uint32_t cameraFrameId {0};
static uint32_t poseId        {0};

//Structure to store camera frame properties.
struct CameraFrameProperties
{
    uint32_t                           numberOfImages {0};
    int32_t                            playbackCount  {0};
    float                              extrinsics[12] {};
    Vuforia::Driver::CameraMode        cameraMode;
    Vuforia::Driver::CameraIntrinsics  cameraIntrinsics;
};
CameraFrameProperties mCameraFrameProperties;

/*-------------------------------------------------------------------------------------------------------------------*/
/* Implement DriverCamera */
/*-------------------------------------------------------------------------------------------------------------------*/

static const char* k_writerEventName = "Local\\WriterEvent";
static const char* k_readerEventName = "Local\\ReaderEvent";
static const char* k_sharedMemoryName = "Local\\SharedMemory";
static const uint32_t k_sharedMemorySize = (1920*1080*4*2);

static uint8_t *CreateSharedMemory(HANDLE* writerEventHandle, HANDLE* readerEventHandle, HANDLE* sharedMemoryHandle)
{
  *writerEventHandle = NULL;
  *readerEventHandle = NULL;
  *sharedMemoryHandle = NULL;

  // Create two events: one that reader waits on and writer signals, and one that writer waits on and reader signals
  HANDLE writerHandle = CreateEventA(NULL, false, false, k_writerEventName);
  *writerEventHandle = writerHandle;
  if (!writerHandle)
  {
    Platform::log(Util::Format() << "Unable to create writer event: code=" << GetLastError());
    goto error;
  }

  HANDLE readerHandle = CreateEventA(NULL, false, false, k_readerEventName);
  *readerEventHandle = readerHandle;
  if (!readerHandle)
  {
    Platform::log(Util::Format() << "Unable to create event: code=" << GetLastError());
    goto error;
  }

  // Create shared memory region and get pointer to it
  HANDLE memoryHandle = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, k_sharedMemorySize, k_sharedMemoryName);
  *sharedMemoryHandle = memoryHandle;
  if (!memoryHandle)
  {
    Platform::log(Util::Format() << "Unable to create mapping object: code=" << GetLastError());
    goto error;
  }
  uint8_t* buffer = (uint8_t*)MapViewOfFile(memoryHandle, FILE_MAP_ALL_ACCESS, 0, 0, k_sharedMemorySize);
  if (!buffer)
  {
    Platform::log(Util::Format() << "Unable to get pointer to file view");
    goto error;
  }

  Platform::log("SUCCESS! Shared memory objects created.");

  return buffer;

error:
  if (*readerEventHandle)
  {
    CloseHandle(*readerEventHandle);
  }

  if (*writerEventHandle)
  {
    CloseHandle(*writerEventHandle);
  }

  if (*sharedMemoryHandle)
  {
    CloseHandle(*sharedMemoryHandle);
  }

  return nullptr;
}

DriverCamera::DriverCamera(Vuforia::Driver::PlatformData* platformData, void* userData)
{
  m_sharedBuffer = CreateSharedMemory(&m_writerEventHandle, &m_readerEventHandle, &m_sharedMemoryHandle);
  /*
  // This code gets the app container named object path
  wchar_t objectPath[1024];
  unsigned long objectPathLength = 0;
  if (!GetAppContainerNamedObjectPath(nullptr, nullptr, 1024, objectPath, &objectPathLength))
  {
    Platform::log("Failed to get app container's named object path!");
  }
  else
  {
    std::wstring ws(objectPath, objectPathLength);
    using convert_type = std::codecvt_utf8<wchar_t>;
    std::wstring_convert<convert_type, wchar_t> converter;
    std::string path = converter.to_bytes(ws);
    Platform::log(Util::Format() << "App contained namer object path: " << path);
  }
  */


    mPlatformData= platformData;
    mUserData = userData;
    //Initialize camera frame parameters. In real world driver implementation, these would be queried from underlying platform/device.
    mCameraFrameProperties.cameraMode.fps =  FPS;
    mCameraFrameProperties.cameraMode.format = Vuforia::Driver::PixelFormat::NV12;
    mCameraFrameProperties.cameraMode.width = 1920;
    mCameraFrameProperties.cameraMode.height = 1080;
    mCameraFrameProperties.cameraIntrinsics.principalPointX = 964.402771f;
    mCameraFrameProperties.cameraIntrinsics.principalPointY = 542.023193f;
    mCameraFrameProperties.cameraIntrinsics.focalLengthX = 1487.305298f;
    mCameraFrameProperties.cameraIntrinsics.focalLengthY = 1485.423584f;
    /*
    mCameraFrameProperties.cameraMode.format = Vuforia::Driver::PixelFormat::NV21;
    mCameraFrameProperties.cameraMode.width = 640;
    mCameraFrameProperties.cameraMode.height = 480;
    mCameraFrameProperties.cameraIntrinsics.principalPointX = 319.880646f;
    mCameraFrameProperties.cameraIntrinsics.principalPointY = 239.459396f;
    mCameraFrameProperties.cameraIntrinsics.focalLengthX = 506.724396f;
    mCameraFrameProperties.cameraIntrinsics.focalLengthY = 504.880859f;
    */
    memcpy(mCameraFrameProperties.cameraIntrinsics.distortionCoefficients, DISTORTION_COEFF, sizeof(DISTORTION_COEFF));
    Platform::log("DriverCamera::DriverCamera");
}

DriverCamera::~DriverCamera()
{
    Platform::log("DriverCamera::~DriverCamera");
}

bool DriverCamera::open()
{
    Platform::log("DriverCamera::open");
    return RefImplSequencePlayer::getInstance().open(this);
}

bool DriverCamera::close()
{
    Platform::log("DriverCamera::close");
    return RefImplSequencePlayer::getInstance().close();
}
bool DriverCamera::start(Vuforia::Driver::CameraMode /* cameraMode */, Vuforia::Driver::CameraCallback* cb)
{
    // Assign the callback that will be used to deliver the frames to Vuforia.
    mEngineCameraCallback = cb;
    Platform::log("DriverCamera::start");
    return RefImplSequencePlayer::getInstance().start();
}

bool DriverCamera::stop()
{
    Platform::log("DriverCamera::stop");
    return RefImplSequencePlayer::getInstance().stop();
}

uint32_t DriverCamera::getNumSupportedCameraModes()
{
    return 1;
}

bool DriverCamera::getSupportedCameraMode(uint32_t  /*index*/ , Vuforia::Driver::CameraMode* out)
{
    out->width = mCameraFrameProperties.cameraMode.width;
    out->height = mCameraFrameProperties.cameraMode.height;
    out->fps = mCameraFrameProperties.cameraMode.fps;
    out->format = mCameraFrameProperties.cameraMode.format;
    return true;
 }

bool DriverCamera::supportsExposureMode(Vuforia::Driver::ExposureMode /* parameter */)
{
    return false;
}

Vuforia::Driver::ExposureMode DriverCamera::getExposureMode()
{
    return Vuforia::Driver::ExposureMode::UNKNOWN;
}

bool DriverCamera::setExposureMode(Vuforia::Driver::ExposureMode /* mode */)
{
    return false;
}

bool DriverCamera::supportsExposureValue()
{
    return false;
}

uint64_t DriverCamera::getExposureValueMin()
{
   return 0;
}

uint64_t DriverCamera::getExposureValueMax()
{
    return 0;
}

uint64_t DriverCamera::getExposureValue()
{
    return 0;
}

bool DriverCamera::setExposureValue(uint64_t /* exposureTime */)
{
    return false;
}

bool DriverCamera::supportsFocusMode(Vuforia::Driver::FocusMode /* parameter */)
{
    return false;
}

Vuforia::Driver::FocusMode DriverCamera::getFocusMode()
{
    return Vuforia::Driver::FocusMode::UNKNOWN;
}

bool DriverCamera::setFocusMode(Vuforia::Driver::FocusMode /* mode */)
{
    return false;
}

bool DriverCamera::supportsFocusValue()
{
    return false;
}

float DriverCamera::getFocusValueMin()
{
    return 0.0f;
}

float DriverCamera::getFocusValueMax()
{
    return 0.0f;
}

float DriverCamera::getFocusValue()
{
    return 0.0f;
}

bool DriverCamera::setFocusValue(float /* value */)
{
    return false;
}

void DriverCamera::invokeEngineCameraCallback(Vuforia::Driver::CameraFrame* inFrame)
{
    if (mEngineCameraCallback)
    {
        inFrame->timestamp = cameraFrameId * FRAME_DELTA;
        mEngineCameraCallback->onNewCameraFrame(inFrame);
    }
}
bool DriverCamera::getCameraFrame(Vuforia::Driver::CameraFrame& cameraFrame, std::vector<uint8_t>& frameBytes)
{
  // Continuously read value and print whenever it changes
  /*
  WaitForSingleObject(m_writerEventHandle, INFINITE);
  uint32_t value = *((uint32_t*)m_sharedBuffer);
  SetEvent(m_readerEventHandle);
  Platform::log(Util::Format() << "From Win32: " << value);
  */

    // Wait for new frame data and then copy it over
    WaitForSingleObject(m_writerEventHandle, INFINITE);
    uint32_t frameLength = *(uint32_t*)&m_sharedBuffer[0];
    if (frameBytes.size() != frameLength)
    {
      frameBytes.resize(frameLength);
    }
    memcpy(frameBytes.data(), &m_sharedBuffer[sizeof(uint32_t)], frameLength);
    SetEvent(m_readerEventHandle);


    uint32_t frameStride = 0;
    cameraFrameId++;

    cameraFrame.timestamp = cameraFrameId * FRAME_DELTA;
    cameraFrame.buffer = frameBytes.data();
    cameraFrame.bufferSize = static_cast<uint32_t>(frameBytes.size());
    cameraFrame.format = mCameraFrameProperties.cameraMode.format;
    cameraFrame.height = mCameraFrameProperties.cameraMode.height;
    cameraFrame.width = mCameraFrameProperties.cameraMode.width;
    cameraFrame.index = cameraFrameId;
    cameraFrame.intrinsics = mCameraFrameProperties.cameraIntrinsics;
    cameraFrame.stride = frameStride;


/*

    if (frameBytes.size() != IMAGE_LENGTH)
    {
        frameBytes.resize(IMAGE_LENGTH);
    }
    memcpy(frameBytes.data(),IMAGE, IMAGE_LENGTH);
    cameraFrame.timestamp = cameraFrameId * FRAME_DELTA;
    cameraFrame.buffer = frameBytes.data();
    cameraFrame.bufferSize = static_cast<uint32_t>(frameBytes.size());
    cameraFrame.format = mCameraFrameProperties.cameraMode.format;
    cameraFrame.height = mCameraFrameProperties.cameraMode.height;
    cameraFrame.width = mCameraFrameProperties.cameraMode.width;
    cameraFrame.index = cameraFrameId;
    cameraFrame.intrinsics = mCameraFrameProperties.cameraIntrinsics;
    cameraFrame.stride = frameStride;

    //control the feed rate.
    if (mCameraFrameProperties.cameraMode.fps)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / mCameraFrameProperties.cameraMode.fps));
    }
*/
    return true;
}

/*-------------------------------------------------------------------------------------------------------------------*/
/* Implement DriverPositionalDeviceTracker */
/*-------------------------------------------------------------------------------------------------------------------*/
DriverPositionalDeviceTracker::DriverPositionalDeviceTracker(Vuforia::Driver::PlatformData* platformData, void* userData)
{
    mPlatformData= platformData;
    mUserData = userData;
    Platform::log("DriverPositionalDeviceTracker::DriverPositionalDeviceTracker");
}

DriverPositionalDeviceTracker::~DriverPositionalDeviceTracker()
{
}

bool DriverPositionalDeviceTracker::open()
{
    Platform::log("DriverPositionalDeviceTracker::open");
    return RefImplSequencePlayer::getInstance().open(this);
}

bool DriverPositionalDeviceTracker::close()
{
    Platform::log("DriverPositionalDeviceTracker::close");
    return RefImplSequencePlayer::getInstance().close();
}

bool DriverPositionalDeviceTracker::start(Vuforia::Driver::PoseCallback* cb, Vuforia::Driver::AnchorCallback*)
{
    // Assign the callback that will be used to deliver poses to Vuforia.
    mEnginePoseCallback = cb;
    Platform::log("DriverPositionalDeviceTracker::start");
    return RefImplSequencePlayer::getInstance().start();
}

bool DriverPositionalDeviceTracker::stop()
{
    Platform::log("DriverPositionalDeviceTracker::stop");
    return RefImplSequencePlayer::getInstance().stop();
}
bool DriverPositionalDeviceTracker::resetTracking()
{
   return false;
}

void DriverPositionalDeviceTracker::invokeEnginePoseCallback(Vuforia::Driver::Pose* pose)
{
    if (mEnginePoseCallback)
    {
        pose->timestamp = poseId * FRAME_DELTA;
        mEnginePoseCallback->onNewPose(pose);
    }
}
bool DriverPositionalDeviceTracker::getPose(Vuforia::Driver::Pose& pose)
{
    constexpr float translation[]  = { 0.0f, 0.0f, 0.0f };
    constexpr float rotation[] = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
    poseId++;
    memcpy(pose.translationData, translation, sizeof(translation));
    memcpy(pose.rotationData, rotation, sizeof(rotation));
    pose.timestamp = poseId * FRAME_DELTA;
    pose.coordinateSystem = Vuforia::Driver::PoseCoordSystem::CAMERA;
    pose.reason = Vuforia::Driver::PoseReason::VALID;
    pose.validity = Vuforia::Driver::PoseValidity::VALID;
    return true;
}

/*-------------------------------------------------------------------------------------------------------------------*/
/* Implement RefDriverImpl */
/*-------------------------------------------------------------------------------------------------------------------*/
RefDriverImpl::RefDriverImpl(Vuforia::Driver::PlatformData* platformData, void* userdata)
    : mPlatformData (platformData) ,
      mUserdata(userdata) ,
      mExternalCamera(nullptr) , mExternalPDT(nullptr)
{
    Platform::log("RefDriverImpl::RefDriverImpl");
}

RefDriverImpl::~RefDriverImpl(){}

Vuforia::Driver::ExternalCamera* VUFORIA_DRIVER_CALLING_CONVENTION RefDriverImpl::createExternalCamera()
{
    if (mExternalCamera == nullptr)
    {
        mExternalCamera = new DriverCamera(mPlatformData, mUserdata);
        Platform::log("createExternalCamera");
        return mExternalCamera;
    }

    // Creating multiple cameras considered an error
    return nullptr;
}

void VUFORIA_DRIVER_CALLING_CONVENTION RefDriverImpl::destroyExternalCamera(Vuforia::Driver::ExternalCamera* instance)
{
    if (instance == mExternalCamera)
    {
        delete static_cast<DriverCamera*>(instance);
        mExternalCamera = nullptr;
        Platform::log("destroyExternalCamera");
    }
}
Vuforia::Driver::ExternalPositionalDeviceTracker* VUFORIA_DRIVER_CALLING_CONVENTION RefDriverImpl::createExternalPositionalDeviceTracker()
{
    if (mExternalPDT == nullptr)
    {
        mExternalPDT = new DriverPositionalDeviceTracker(mPlatformData, mUserdata);
        Platform::log("createExternalPositionalDeviceTracker");
        return mExternalPDT;
    }
    // Creating multiple external positional device trackers is considered as an error.
    return nullptr;
}
void VUFORIA_DRIVER_CALLING_CONVENTION RefDriverImpl::destroyExternalPositionalDeviceTracker(Vuforia::Driver::ExternalPositionalDeviceTracker* instance)
{
    if (instance == mExternalPDT)
    {
        delete static_cast<DriverPositionalDeviceTracker*>(instance);
        mExternalPDT = nullptr;
        Platform::log("destroyExternalPositionalDeviceTracker");
    }
}

uint32_t VUFORIA_DRIVER_CALLING_CONVENTION RefDriverImpl::getCapabilities()
{
    Platform::log("RefDriverImpl::getCapabilities");
    return (uint32_t)(Vuforia::Driver::Capability::CAMERA_IMAGE | Vuforia::Driver::Capability::CAMERA_POSE);
}
