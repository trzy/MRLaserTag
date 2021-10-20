import asyncio
from functools import reduce

import pywintypes   # must be imported first: https://stackoverflow.com/questions/58631512/pywin32-and-python-3-8-0
import win32event
from mmapfile import mmapfile


class VuforiaForwarder:
    """
    Subscribes to RGB frames and forwards them to the Vuforia app using shared
    memory.
    """

    def __init__(self):
        # Vuforia driver shared memory
        self._reader_handle = None
        self._writer_handle = None
        self._memory_handle = None

    async def on_rgb_frame(self, device_id, rgb_data, color_frame):
        handles = self._get_shared_memory()
        if not handles:
            return

        reader_handle, writer_handle, memory_handle = handles
        raw_bytes = rgb_data.getData().tobytes(order = 'C')
        if len(raw_bytes) == int(1920 * 1080 * 1.5):
            # Write data
            memory_handle.seek(0, 0)
            memory_handle.write(len(raw_bytes).to_bytes(4, byteorder = "little"))
            memory_handle.write(raw_bytes)

            # Signal that reader (Vuforia driver) may proceed
            win32event.SetEvent(writer_handle)

            # Wait for the reader to finish before writing again
            # TODO: if Vuforia driver dies, we may get stuck here. Maybe we
            #       should retain state and resume when called again on
            #       subsequent frame.
            timeout_ms = 32
            while win32event.WaitForSingleObject(reader_handle, timeout_ms) != win32event.WAIT_OBJECT_0:
                await asyncio.sleep(0)  # let other tasks run while we wait

    def _get_shared_memory(self):
        reader_name = "AppContainerNamedObjects\\S-1-15-2-3950821806-2383410207-2205885981-1193157422-352231163-3160459235-648191853\\ReaderEvent"
        writer_name = "AppContainerNamedObjects\\S-1-15-2-3950821806-2383410207-2205885981-1193157422-352231163-3160459235-648191853\\WriterEvent"
        memory_name = "AppContainerNamedObjects\\S-1-15-2-3950821806-2383410207-2205885981-1193157422-352231163-3160459235-648191853\\SharedMemory"
        memory_size = 1920 * 1080 * 4 * 2

        # Try to get each of our required handles
        if self._reader_handle is None:
            try:
                self._reader_handle = win32event.OpenEvent(win32event.EVENT_ALL_ACCESS, False, reader_name)
            except pywintypes.error as error:
                print("Failed to obtain reader event handle: %s" % error.strerror)
        if self._writer_handle is None:
            try:
                self._writer_handle = win32event.OpenEvent(win32event.EVENT_ALL_ACCESS, False, writer_name)
            except pywintypes.error as error:
                print("Failed to obtain writer event handle: %s" % error.strerror)
        if self._memory_handle is None:
            try:
                self._memory_handle = mmapfile(None, memory_name, memory_size, 0, 0)
            except pywintypes.error as error:
                print("Failed to obtain shared memory handle: %s" % error.strerror)

        ipc_initialized = reduce(lambda x, y: (x is not None) & (y is not None), [ self._reader_handle, self._writer_handle, self._memory_handle ])
        return (self._reader_handle, self._writer_handle, self._memory_handle) if ipc_initialized else None