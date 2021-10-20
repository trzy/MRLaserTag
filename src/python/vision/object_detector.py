import asyncio
import open3d as o3d

import src.python.vision.rgbd as rgbd
from src.python.vision.classifier import Classifier
from src.python.networking.messages import *


class ObjectDetector:
    def __init__(self, calibration):
        _, _, K_rgb_scaled = rgbd.compute_depth_to_rgb_conversion_matrices(calibration = calibration)
        self._classifier = Classifier(intrinsic_matrix = K_rgb_scaled)  # use scaled matrix because RGBD frame is 1280x720
        self._color_frame = None
        self._class_filter = [ "couch", "book", "dining table", "bed", "suitcase", "chair", "vase", "potted plant" ]
        self._camera_task = None
        self._text_label = None
        self._labels_and_points = []

    async def run(self, camera_task):
        self._camera_task = camera_task

        # Detection is run once after an N-second count-down
        self._text_label = camera_task.createTextLabel()
        for i in range(10):
            self._text_label.text = "Object detection in %d" % (10 - i)
            await asyncio.sleep(1)

        # Subscribe for frames
        self._camera_task.add_rgb_subscriber(self)
        self._camera_task.add_depth_subscriber(self)

        # Pause for a while to show completion message
        await asyncio.sleep(5)
        del self._text_label    # remove label from window

    async def on_rgb_frame(self, device_id, rgb_data, color_frame):
        self._color_frame = color_frame

    async def on_depth_frame(self, device_id, depth_frame, point_cloud):
        if self._color_frame is not None:
            labels_and_points = await self._classifier.classify(color_frame = self._color_frame, depth_frame = depth_frame)
            labels_and_points = [ (label, points) for label, points in labels_and_points if label in self._class_filter ]
            if self._text_label is not None:
                self._text_label.text = "Object detection found %d relevant objects" % len(labels_and_points)
            if self._camera_task is not None:
                self._camera_task.remove_rgb_subscriber(self)
                self._camera_task.remove_depth_subscriber(self)
            self._labels_and_points = labels_and_points
            self._color_frame = None

    def get_transformed_boxes(self, oakd_to_hmd_matrix):
        labels_and_points = [ (label, self._transform_points_to_hmd_space(points = points.copy(), transform = oakd_to_hmd_matrix)) for label, points in self._labels_and_points ]
        boxes = self._extract_objects(labels_and_points = labels_and_points)

        #for i in range(len(labels_and_points)):
        #    if labels_and_points[i][0] == "suitcase":
        #        # Dump to file for debugging
        #        hmd_points = labels_and_points[i][1]
        #        #from src.python.vision.rgbd import _dump_point_cloud
        #        #_dump_point_cloud(hmd_points.T)
        #        break

        return boxes

    @staticmethod
    def _transform_points_to_hmd_space(points, transform):
        points[1,:] *= -1.0 # invert y to convert from RHS -> LHS
        points = np.vstack([ points, np.ones(points.shape[1]) ])    # point_cloud [3,N] -> [4,N], with w = 1.0
        hmd_points = np.dot(transform, points)                      # transform each point
        return hmd_points[0:3,:]                                    # [3,N]

    @staticmethod
    def _extract_objects(labels_and_points):
        boxes = []
        for label, points in labels_and_points:
            if points.size < 8: # make sure we have a sufficient number of points to fit a box (need 3 or 4 and non-coplanar)
                continue

            # Who's down with OBB? Yeah, you know me!
            points = o3d.utility.Vector3dVector(points.transpose())
            obb = o3d.geometry.OrientedBoundingBox.create_from_points(points)

            pose_matrix = np.hstack([ obb.R, np.expand_dims(obb.center, 1) ])
            pose_matrix = np.vstack([ pose_matrix, np.array([ 0, 0, 0, 1]) ])

            boxes.append(Box(label = label, pose_matrix = pose_matrix, extents = obb.extent))
        return boxes
