# include pywin32 in requirements.txt
#>>> import pywintypes # must be imported first: https://stackoverflow.com/questions/58631512/pywin32-and-python-3-8-0
#>>> import win32event
#>>> from mmapfile import mmapfile
#>>> event=win32event.OpenEvent(win32event.EVENT_ALL_ACCESS, False, "Local\\WriterEvent")
#>>> revent=win32event.OpenEvent(win32event.EVENT_ALL_ACCESS, False, "Local\\ReaderEvent")
#>>> fp=mmapfile(None, "Local\\MyFileMappingObject", 256, 0, 256)
#>>> bytes=b'0x1234'
#>>> bytes=b'0x1234'


import asyncio
from collections import deque
import depthai as dai
import numpy as np
import sys
import time

import pywintypes   # must be imported first: https://stackoverflow.com/questions/58631512/pywin32-and-python-3-8-0
import win32event
from mmapfile import mmapfile


###############################################################################
# IPC Task
###############################################################################

g_ipc_initialized = False

async def init_ipc():
    global g_ipc_initialized
    global g_reader_event_handle
    global g_writer_event_handle
    global g_memory_handle

    g_reader_event_handle = None
    g_writer_event_handle = None
    g_memory_handle = None

    reader_event_name = "AppContainerNamedObjects\\S-1-15-2-3950821806-2383410207-2205885981-1193157422-352231163-3160459235-648191853\\ReaderEvent"
    writer_event_name = "AppContainerNamedObjects\\S-1-15-2-3950821806-2383410207-2205885981-1193157422-352231163-3160459235-648191853\\WriterEvent"
    shared_memory_name = "AppContainerNamedObjects\\S-1-15-2-3950821806-2383410207-2205885981-1193157422-352231163-3160459235-648191853\\SharedMemory"
    shared_memory_size = 1920 * 1080 * 4 * 2

    while not g_ipc_initialized:
        # Try to get each of the handles we need
        if g_reader_event_handle is None:
            try:
                g_reader_event_handle = win32event.OpenEvent(win32event.EVENT_ALL_ACCESS, False, reader_event_name)
            except pywintypes.error as error:
                print("Failed to obtain reader event handle: %s" % error.strerror)
        if g_writer_event_handle is None:
            try:
                g_writer_event_handle = win32event.OpenEvent(win32event.EVENT_ALL_ACCESS, False, writer_event_name)
            except pywintypes.error as error:
                print("Failed to obtain writer event handle: %s" % error.strerror)
        if g_memory_handle is None:
            try:
                g_memory_handle = mmapfile(None, shared_memory_name, shared_memory_size, 0, 0)
            except pywintypes.error as error:
                print("Failed to obtain shared memory handle: %s" % error.strerror)
        
        g_ipc_initialized = (g_reader_event_handle is not None) and (g_writer_event_handle is not None) and (g_memory_handle is not None)
        if not g_ipc_initialized:
            print("Retrying in 1 second...")
        await asyncio.sleep(1)

    print("Established IPC link to Vuforia camera driver")


###############################################################################
# Camera Task
#
# Acquires frames and performs AprilTag detection.
###############################################################################

def create_rgb_pipeline():
    pipeline = dai.Pipeline()
    cam_rgb = pipeline.createColorCamera()
    xout_video = pipeline.createXLinkOut()
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(True)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    xout_video.setStreamName("rgb_video")
    cam_rgb.video.link(xout_video.input)
    return pipeline

async def send_frames_to_vuforia_app(device):
    global g_ipc_initialized
    global g_reader_event_handle
    global g_writer_event_handle
    global g_memory_handle
    
    lastFrameTime = time.perf_counter()
    queue = device.getOutputQueue("rgb_video", 8, blocking = False)
    
    while True:
        if g_ipc_initialized:
            # Acquire frame
            feed_name = queue.getName()
            frame_data = queue.get()
            raw_bytes = frame_data.getData().tobytes(order = 'C')

            # Frame timing
            now = time.perf_counter()
            frameRateHz = 1.0 / (now - lastFrameTime)
            lastFrameTime = now
            
            # If we have data, send it to the camera driver
            if len(raw_bytes) == int(1920 * 1080 * 1.5):
                # Write data
                g_memory_handle.seek(0, 0)
                g_memory_handle.write(len(raw_bytes).to_bytes(4, byteorder = "little"))
                g_memory_handle.write(raw_bytes)
                print("Wrote %d bytes (%1.1f Hz)" % (len(raw_bytes), frameRateHz))

                # Signal that reader (Vuforia driver) may proceed 
                win32event.SetEvent(g_writer_event_handle)

                # Wait for the reader to finish before writing again
                timeout_ms = 32
                while win32event.WaitForSingleObject(g_reader_event_handle, timeout_ms) != win32event.WAIT_OBJECT_0:
                    await asyncio.sleep(0)  # let other tasks run while we wait

        # Release timeslice
        await asyncio.sleep(0)

async def run_camera(loop):
    pipeline = create_rgb_pipeline()
    with dai.Device(pipeline) as device:
        device.startPipeline()
        await send_frames_to_vuforia_app(device = device)
    loop.stop() # tear down the entire app
    print("Finished")


###############################################################################
# Program Entry Point
###############################################################################

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.create_task(init_ipc())
    loop.create_task(run_camera(loop = loop))
    loop.run_forever()