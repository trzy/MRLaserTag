/*===============================================================================
Copyright (c) 2020 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/
#ifndef _REF_DRIVER_IMPL_H_
#define _REF_DRIVER_IMPL_H_

#include <string>
#include <vector>
#include <Vuforia/Driver/Driver.h>

#include <windows.h>

/*-------------------------------------------------------------------------------------------------------------------*/
//Implement DriverCamera by deriving from Vuforia::Driver::ExternalCamera to feed camera frames into Vuforia Engine.
/*-------------------------------------------------------------------------------------------------------------------*/
class DriverCamera final : public Vuforia::Driver::ExternalCamera
{
public:
    DriverCamera(Vuforia::Driver::PlatformData* platformData, void* userData);
    virtual ~DriverCamera();

    //Override Vuforia::Driver::ExternalCamera members.
    bool VUFORIA_DRIVER_CALLING_CONVENTION open() override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION close()  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION start(Vuforia::Driver::CameraMode /* cameraMode */, Vuforia::Driver::CameraCallback* cb)  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION stop()  override;
    uint32_t VUFORIA_DRIVER_CALLING_CONVENTION getNumSupportedCameraModes()  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION getSupportedCameraMode(uint32_t  index , Vuforia::Driver::CameraMode* out)  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION supportsExposureMode(Vuforia::Driver::ExposureMode /* parameter */)  override;
    Vuforia::Driver::ExposureMode VUFORIA_DRIVER_CALLING_CONVENTION getExposureMode()  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION setExposureMode(Vuforia::Driver::ExposureMode /* mode */)  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION supportsExposureValue()  override;
    uint64_t VUFORIA_DRIVER_CALLING_CONVENTION getExposureValueMin()  override;
    uint64_t VUFORIA_DRIVER_CALLING_CONVENTION getExposureValueMax()  override;
    uint64_t VUFORIA_DRIVER_CALLING_CONVENTION getExposureValue()  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION setExposureValue(uint64_t /* exposureTime */)  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION supportsFocusMode(Vuforia::Driver::FocusMode /* parameter */)  override;
    Vuforia::Driver::FocusMode VUFORIA_DRIVER_CALLING_CONVENTION getFocusMode()  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION setFocusMode(Vuforia::Driver::FocusMode /* mode */)  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION supportsFocusValue()  override;
    float VUFORIA_DRIVER_CALLING_CONVENTION getFocusValueMin()  override;
    float  VUFORIA_DRIVER_CALLING_CONVENTION getFocusValueMax()  override;
    float VUFORIA_DRIVER_CALLING_CONVENTION getFocusValue()  override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION setFocusValue(float /* value */)  override;
    //Invokes engine callback to feed camera frames.
    void invokeEngineCameraCallback(Vuforia::Driver::CameraFrame* inFrame);
    //Returns the next camera frame.
    bool getCameraFrame(Vuforia::Driver::CameraFrame&, std::vector<uint8_t>&);

private:
    Vuforia::Driver::CameraCallback*    mEngineCameraCallback{ nullptr };
    Vuforia::Driver::PlatformData*      mPlatformData        { nullptr };
    void*                               mUserData            { nullptr };

    HANDLE m_writerEventHandle;
    HANDLE m_readerEventHandle;
    HANDLE m_sharedMemoryHandle;
    uint8_t* m_sharedBuffer;
};

/*---------------------------------------------------------------------------------------------------------------------------------------------*/
//Implement DriverPositionalDeviceTracker by deriving from Vuforia::Driver::ExternalPositionalDeviceTracker to feed poses into Vuforia Engine.
/*---------------------------------------------------------------------------------------------------------------------------------------------*/
class DriverPositionalDeviceTracker final : public Vuforia::Driver::ExternalPositionalDeviceTracker
{
public:
    DriverPositionalDeviceTracker(Vuforia::Driver::PlatformData* platformData, void* userData);
    virtual ~DriverPositionalDeviceTracker();

    //Override Vuforia::Driver::ExternalPositionalDeviceTracker members.
    bool VUFORIA_DRIVER_CALLING_CONVENTION open() override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION close() override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION start(Vuforia::Driver::PoseCallback* cb, Vuforia::Driver::AnchorCallback* acb = nullptr) override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION stop() override;
    bool VUFORIA_DRIVER_CALLING_CONVENTION resetTracking() override;
    //Invokes engine callback to feed poses.
    void invokeEnginePoseCallback(Vuforia::Driver::Pose* pose);
    //Returns the next pose.
    bool getPose(Vuforia::Driver::Pose&);

private:
    Vuforia::Driver::PoseCallback*  mEnginePoseCallback  { nullptr };
    Vuforia::Driver::PlatformData*  mPlatformData        { nullptr };
    void*                           mUserData            { nullptr };
};

/*-------------------------------------------------------------------------------------------------------------------*/
//RefDriverImpl implements the VuforiaDriver base class.
//This class is used for constructing and destroying the ExternalCamera and ExternalPositionalDeviceTracker source objects.
//The documentation of the public methods can be found in Vuforia/Driver/Driver.h header.
/*-------------------------------------------------------------------------------------------------------------------*/
class RefDriverImpl final : public Vuforia::Driver::VuforiaDriver
{
public:
    RefDriverImpl(Vuforia::Driver::PlatformData* platformData, void* userdata);
    virtual ~RefDriverImpl();

    //Override Vuforia::Driver::VuforiaDriver members.
    Vuforia::Driver::ExternalCamera* VUFORIA_DRIVER_CALLING_CONVENTION createExternalCamera() override;
    void VUFORIA_DRIVER_CALLING_CONVENTION destroyExternalCamera(Vuforia::Driver::ExternalCamera* instance) override;
    Vuforia::Driver::ExternalPositionalDeviceTracker* VUFORIA_DRIVER_CALLING_CONVENTION createExternalPositionalDeviceTracker() override;
    void VUFORIA_DRIVER_CALLING_CONVENTION destroyExternalPositionalDeviceTracker(Vuforia::Driver::ExternalPositionalDeviceTracker* instance) override;
    uint32_t VUFORIA_DRIVER_CALLING_CONVENTION getCapabilities() override;

private:
    Vuforia::Driver::PlatformData*          mPlatformData   { nullptr };
    void*                                   mUserdata       { nullptr };
    DriverCamera*                           mExternalCamera { nullptr };
    DriverPositionalDeviceTracker*          mExternalPDT    { nullptr };
};

#endif // _REF_DRIVER_IMPL_H_
