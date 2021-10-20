import asyncio
import cv2
from functools import reduce
import json
import numpy as np
import os
import open3d as o3d

import pywintypes
import win32event
from mmapfile import mmapfile

from src.python.vision.rgbd import homogeneous_pixel_coordinates


class Classifier:
    """
    Accepts RGBD classification requests and pipes them to the classifier
    server.
    """  
    def __init__(self, intrinsic_matrix):
        self._K_rgb = intrinsic_matrix
        self._server_event_handle = None
        self._client_event_handle = None
        self._memory_handle = None
        self._client_id = os.getpid()

    async def classify(self, color_frame, depth_frame):
        """
        Returns
        -------
        List[(str, np.ndarray)]
            An array of tuples containing the instance label and points (shape
            [3,N]).
        """
        handles = self._get_shared_memory()
        if not handles:
            return []   # TODO: wait for some time?
        server_event_handle, client_event_handle, memory_handle = handles

        # Convert color frame from BGR to RGB format and resize to 1280x720
        resized = cv2.resize(src = color_frame, dsize = ((1280, 720)))
        rgb_bytes = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).tobytes(order = "C")
        del resized
        
        # Convert depth frame to point cloud that corresponds 1:1 to flattened color frame
        point_cloud_bytes = self._convert_depth_frame_to_point_cloud(depth_frame = depth_frame).tobytes(order = "C")

        # Write to shared memory
        print("Sending classification job...")
        memory_handle.seek(0, 0)
        memory_handle.write(int(self._client_id).to_bytes(4, byteorder = "little")) # TODO: will be used to support multiple clients in the future
        memory_handle.write(int(0).to_bytes(4, byteorder = "little"))               # TODO: completion flag will be used to support multiple clients in the future
        memory_handle.write(len(rgb_bytes).to_bytes(4, byteorder = "little"))
        memory_handle.write(rgb_bytes)
        memory_handle.write(len(point_cloud_bytes).to_bytes(4, byteorder = "little"))
        memory_handle.write(point_cloud_bytes)
        
        # Signal that classifier server may proceed
        win32event.SetEvent(server_event_handle)

        # Wait for server to complete its work
        # TODO: if server dies, we will get stuck here. Need to be more robust.
        timeout_ms = 32
        while win32event.WaitForSingleObject(client_event_handle, timeout_ms) != win32event.WAIT_OBJECT_0:
            await asyncio.sleep(0)  # let other tasks run while we wait

        # Read back data
        memory_handle.seek(0, 0)
        client_id = int.from_bytes(bytes = self._memory_handle.read(4), byteorder = "little")
        completed = int.from_bytes(bytes = self._memory_handle.read(4), byteorder = "little")
        assert client_id == self._client_id 
        assert completed != 0
        num_labels_bytes = int.from_bytes(bytes = self._memory_handle.read(4), byteorder = "little")
        labels_bytes = self._memory_handle.read(num_labels_bytes)
        labels = json.loads(labels_bytes.decode("utf-8"))
        labels_and_points = []
        if len(labels) > 0:
            print("Classification Results")
            print("----------------------")
            max_label_length = max([ len(label) for label in labels ])
            for i in range(len(labels)):
                num_points_bytes = int.from_bytes(bytes = self._memory_handle.read(4), byteorder = "little")
                points_bytes = self._memory_handle.read(num_points_bytes)
                points = np.frombuffer(buffer = points_bytes, dtype = float).reshape((3, -1))   # reshape to [3,N], whatever N was
                points = self._cull_extrema(points = points)
                points = self._cull_outliers(points = points)
                print("  %s: %s" % (labels[i].ljust(max_label_length, " "), str(points.shape)))
                if points.size > 0:
                    labels_and_points.append((labels[i], points))
        return labels_and_points

    def _convert_depth_frame_to_point_cloud(self, depth_frame):
        points = homogeneous_pixel_coordinates(1280, 720)
        K_inverse = np.linalg.inv(self._K_rgb)
        points_3d = (
            np.dot(K_inverse, points) * depth_frame.flatten() * 0.1 # convert to cm, which is how camera was calibrated
        )
        return points_3d * 1e-2 # to meters

    def _cull_outliers(self, points):
        # This would be sufficient to exclude outliers but it crashes frequently
        #pcd = o3d.geometry.PointCloud()
        #pcd.points = o3d.utility.Vector3dVector(points.T)
        #pcd, idxs = pcd.remove_radius_outlier(nb_points = 30, radius = 5e-2)
        #pcd = pcd.select_by_index(idxs)
        #points = np.asarray(pcd.points).T

        # Downsampling seems to work...
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points.T)
        pcd = pcd.voxel_down_sample(voxel_size = 0.08)
        points = np.asarray(pcd.points).T

        # ...and gets us to a point where we can do a simple O(N^2) culling.
        # In a dense cloud, we might want to have a small radius threshold and
        # look for a large number of points within but when downsampled, we
        # will have fewer points separate by the voxelization distance. Hence,
        # rather than searching for e.g., 30 points within a 10cm radius, we
        # look for 30 points within a 1m radius.
        radius2_threshold = 1 * 1
        neighbors_threshold = 30
        inlier_idxs = []
        for i in range(points.shape[1]):
            num_within_range = 0
            p1 = points[:,i]
            for j in range(points.shape[1]):
                p2 = points[:,j]
                delta = p1 - p2
                r2 = np.sum(delta * delta)
                if r2 < radius2_threshold:
                    num_within_range += 1
            if num_within_range >= neighbors_threshold:
                inlier_idxs.append(i)
        return points[:,inlier_idxs]

    def _cull_extrema(self, points):
        """
        Remove extreme outliers and garbage depth values.
        """
        distance_threshold = 5
        threshold_x = np.quantile(points[0,:], 0.9)
        threshold_y = np.quantile(points[1,:], 0.9)
        threshold_z = np.quantile(points[2,:], 0.9)
        idxs_x = np.where((points[0,:] < threshold_x) & (points[0,:] < distance_threshold))
        points = points[:, idxs_x[0]]
        idxs_y = np.where((points[1,:] < threshold_y) & (points[1,:] < distance_threshold))
        points = points[:, idxs_y[0]]
        idxs_z = np.where((points[2,:] < threshold_z) & (points[2,:] < distance_threshold))
        points = points[:, idxs_z[0]]
        return points

    def _get_shared_memory(self):
        server_event_name = "Local\\MRLaserTag_Classifier_Producer" # server waits on client to pulse this to obtain job
        client_event_name = "Local\\MRLaserTag_Classifier_Consumer" # client waits on server to pulse this to obtain result of last job
        memory_name = "Local\\MRLaserTag_Classifier_Memory"
        memory_size = 4*4 + 1280*720*3*1 + 1280*720*3*8

        # Try to get each of the required handles
        if self._server_event_handle is None:
            try:
                self._server_event_handle = win32event.OpenEvent(win32event.EVENT_ALL_ACCESS, False, server_event_name)
                print("Got producer event")
            except pywintypes.error as error:
                print("Failed to obtain server event handle: %s" % error.strerror)
        if self._client_event_handle is None:        
            try:
                self._client_event_handle = win32event.OpenEvent(win32event.EVENT_ALL_ACCESS, False, client_event_name)
                print("Got consumer event")
            except pywintypes.error as error:
                print("Failed to obtain client event handle: %s" % error.strerror)
        if self._memory_handle is None:
            try:
                self._memory_handle = mmapfile(None, memory_name, memory_size, 0, 0)
                print("Got shared memory")
            except pywintypes.error as error:
                print("Failed to obtain shared memory handle: %s" % error.strerror)
            
        ipc_initialized = reduce(lambda x, y: (x is not None) & (y is not None), [ self._server_event_handle, self._client_event_handle, self._memory_handle ])
        return (self._server_event_handle, self._client_event_handle, self._memory_handle) if ipc_initialized else None