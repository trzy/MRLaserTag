/*===============================================================================
Copyright (c) 2020 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/
#ifndef _REF_IMPL_SEQ_PLAYER_H_
#define _REF_IMPL_SEQ_PLAYER_H_

#include <functional>
#include <mutex>
#include <thread>
#include <vector>
#include <Vuforia/Driver/Driver.h>

//Forwrd declarations for Driver camera and positional device tracker.
class DriverCamera;
class DriverPositionalDeviceTracker;

/**
 * This class will call into DriverCamera to get the camera frame parameters and facilitates
 * feeding camera frames into Vuforia engine.
 *
 * This class will call into DriverPositionalDeviceTracker to get pose parameters and facilitates
 * feeding poses into Vuforia engine.
 */
class RefImplSequencePlayer
{
public:
    RefImplSequencePlayer(const RefImplSequencePlayer&) = delete;
    RefImplSequencePlayer(RefImplSequencePlayer&&) = delete;
    RefImplSequencePlayer& operator=(const RefImplSequencePlayer&) = delete;
    RefImplSequencePlayer& operator=(RefImplSequencePlayer &&) = delete;

    static RefImplSequencePlayer& getInstance()
    {
        static RefImplSequencePlayer instance;
        return instance;
    }

    /// Open the camera.
    /**
     * Prepares the camera feed for playback.
     */
    bool open(DriverCamera* camera);

    /// Open the external pose provider.
    /**
     * Prepare poses for playback.
     */
    bool open(DriverPositionalDeviceTracker* pdt);

    /// Start the camera/pose feed.
    bool start();

    /// Stop the camera/pose feed.
    bool stop();

    /// Close the camera/ external pose provider.
    bool close();

private:
    RefImplSequencePlayer() = default;
    void threadFunction();

    bool                                          mWaiting { true };
    bool                                          mIsClosed{ false };
    bool                                          mHoldPose{ false };

    //uint32_t                                      mCurrentFrameIndex{ 0 };
    std::thread                                   mPlayingThread;
    std::mutex                                    mWaitingMutex;
    std::mutex                                    mRewindMutex;
    std::condition_variable                       mPlayingConditionVar;

    DriverCamera*                                 mCamera {nullptr};
    DriverPositionalDeviceTracker*                mPDT    {nullptr};

};

#endif // _REF_IMPL_SEQ_PLAYER_H_
