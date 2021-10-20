/*===============================================================================
Copyright (c) 2019 PTC Inc. All Rights Reserved.

Vuforia is a trademark of PTC Inc., registered in the United States and other
countries.
===============================================================================*/
#include "RefImplSequencePlayer.h"
#include "RefDriverImpl.h"
#include "Platform.h"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>
#include <thread>
#include <stdlib.h>

//Thread to get frame/pose from DriverCamera & DriverPositionalDeviceTracker respectively
void RefImplSequencePlayer::threadFunction()
{
    bool bGotPose = false;
    bool bGotCameraFrame = false;
    Vuforia::Driver::Pose        mPose;
    Vuforia::Driver::CameraFrame mFrame;
    std::vector<uint8_t> frameBytes;

    while (true)
    {
        {
            std::unique_lock<decltype(mWaitingMutex)> lock(mWaitingMutex);
            mPlayingConditionVar.wait(lock, [this] { return !mWaiting; });
        }
        if (mIsClosed)
        {
            break;
        }
        {
            //acquire the mutex to ensure sequence rewind doesn't interfere with frame / pose retrieval.
            std::lock_guard<std::mutex> lock(mRewindMutex);
            //Make sure we are not holding onto previous pose before pulling in new pose from the sequence file.
            if(mPDT && !mHoldPose)
            {
                bGotPose = mPDT->getPose(mPose);
            }
            //Retrieve the camera frame.
            if(mCamera)
            {
                bGotCameraFrame = mCamera->getCameraFrame(mFrame, frameBytes);
            }
        }
        //Check if pose and camera frame were retrieved successfully
        if(mPDT && bGotPose && bGotCameraFrame)
        {
            // If pose timestamp same as camera timestamp, go ahead and send the pose into Vuforia.
            if(mPose.timestamp == mFrame.timestamp)
            {
                mPDT->invokeEnginePoseCallback(&mPose);
                mHoldPose = false;
            }
            //Possible pose hole in the sequence as we got the pose with TS > camera TS
            //Hold on to this pose till we get a matching camera frame.
            else if(mPose.timestamp > mFrame.timestamp)
            {
                mHoldPose = true;
            }
        }
        if(bGotCameraFrame)
        {
            // send the camera frame into Vuforia
            mCamera->invokeEngineCameraCallback(&mFrame);
        }
        if(!bGotCameraFrame && !bGotPose && !mHoldPose)
        {
            //do not exit the thread here. This thread will exit when mIsClosed is set to true ( when RefImplSequencePlayer::close is invoked)
            mWaiting = true;
        }
    }
}

bool RefImplSequencePlayer::open(DriverCamera* camera)
{
    mCamera = camera;
    /*
    * Start the thread when mPDT is null(camera only driver capability) or for following scenario.
    * When the app is put to background, stop and close is called on RefImplSequencePlayer.
    * When the app is coming to the foreground, mPDT is valid but mIsClosed == true.
    * Need to start the thread again and reset mIsClosed.
    */
    if(!mPDT || mIsClosed)
    {
        mIsClosed = false;
        mPlayingThread = std::thread(&RefImplSequencePlayer::threadFunction, this);
    }
    return true;
}

bool RefImplSequencePlayer::open(DriverPositionalDeviceTracker* pdt)
{
    mPDT = pdt;
    //if mCamera is not null, open must have been called before and mPlayingThread is already instantiated.
    if(!mCamera)
    {
        mPlayingThread = std::thread(&RefImplSequencePlayer::threadFunction, this);
    }
    return true;
}

bool RefImplSequencePlayer::start()
{
    std::lock_guard<std::mutex> lock(mWaitingMutex);
    mWaiting = false;
    Platform::log("RefImplSequencePlayer::start");

    mPlayingConditionVar.notify_one();

    return true;
}

bool RefImplSequencePlayer::stop()
{
    std::lock_guard<std::mutex> lock(mWaitingMutex);
    mWaiting = true;
    Platform::log("RefImplSequencePlayer::stop");
    return true;
}

bool RefImplSequencePlayer::close()
{
    mIsClosed = true;
    Platform::log("RefImplSequencePlayer::close");
    if (mPlayingThread.joinable())
    {
        // Must set mWaiting to false in-order to resolve deadlock
        {
            std::unique_lock<std::mutex> lock(mWaitingMutex);
            mWaiting = false;
        }

        mPlayingConditionVar.notify_one();
        mPlayingThread.join();
    }
    return true;
}
