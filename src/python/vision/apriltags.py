import asyncio
import cv2
import numpy as np

from src.python.networking.messages import AprilTagDetection
import thirdparty.apriltag.python.apriltag as apriltag


class AprilTagDetector:
    def __init__(self, K_rgb):
        self.on_apriltags_detected = None
        self.detections = []    # will hold most recent detections as List[AprilTagDetection]

        self._accepting_frames = True

        self._tag_size_by_id = {
            0: 17.5 * 2.54, # inches -> cm  (calibration board)
            1: 15.3,        # cm            (wall)
            2: 43.7         # cm            (wall)
        }

        self._K_rgb = K_rgb
        self._camera_params = [
            K_rgb[0, 0],    # fx
            K_rgb[1, 1],    # fy
            K_rgb[0, 2],    # cx
            K_rgb[1, 2]     # cy
        ]
        self._rotate_z_180 = np.array([ # matrix to rotate about Z axis by 180 degrees
            [ -1,  0, 0, 0 ],
            [  0, -1, 0, 0 ],
            [  0,  0, 1, 0 ],
            [  0,  0, 0, 1 ]
        ])

        options = apriltag.DetectorOptions(quad_contours = False, refine_pose = True, quad_decimate = 0)
        self._detector = apriltag.Detector(options = options, searchpath = ["./bin/"])

    async def run(self, camera_task):
        # Subscribe for frames automatically
        camera_task.add_rgb_subscriber(self)

        # Detection happens for a certain number of seconds
        text_label = camera_task.createTextLabel()
        self._accepting_frames = True
        for i in range(10):
            text_label.text = "AprilTag detection in %d" % (10 - i)
            await asyncio.sleep(1)
        self._accepting_frames = False
        text_label.text = "AprilTag detection found %d tags" % len(self.detections)
        await asyncio.sleep(5)
        del text_label  # remove label from window

    async def on_rgb_frame(self, device_id, rgb_data, color_frame):
        if not self._accepting_frames:
            return

        gray_frame = cv2.cvtColor(color_frame, cv2.COLOR_RGB2GRAY)
        detections = self._detector.detect(img=gray_frame)
        apriltag_detections = []
        for detection in detections:
            # Draw 2D outline
            for i in range(4):
                start, end = detection.corners[i], detection.corners[(i + 1) % 4]
                line_color = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)][i]  # red, green, blue, yellow (OpenCV uses BGR)
                cv2.line(
                    img = color_frame,
                    pt1 = (int(start[0]), int(start[1])),
                    pt2 = (int(end[0]), int(end[1])),
                    color = line_color,
                    thickness = 3
                )

            # Look up tag size
            tag_size = self._tag_size_by_id[detection.tag_id]

            # Get pose
            pose, init_error, final_error = self._detector.detection_pose(
                detection = detection, camera_params = self._camera_params, tag_size = tag_size
            )
            pose = np.array(pose)
            #if detection.tag_id == 0:
            #    print("device_id=%s, id=%d, position=%f,%f,%f" % (device_id, detection.tag_id, pose[0,3], pose[1,3], pose[2,3]))
            #    print("pose=np.array([")
            #    print("  [ %f, %f, %f, %f ]," % (pose[0,0], pose[0,1], pose[0,2], pose[0,3]))
            #    print("  [ %f, %f, %f, %f ]," % (pose[1,0], pose[1,1], pose[1,2], pose[1,3]))
            #    print("  [ %f, %f, %f, %f ]," % (pose[2,0], pose[2,1], pose[2,2], pose[2,3]))
            #    print("  [ %f, %f, %f, %f ]" % (pose[3,0], pose[3,1], pose[3,2], pose[3,3]))
            #    print("])")


            # Convert to LHS coordinate system. I don't fully understand why the 180 degree
            # rotation is needed. Raw AprilTag poses need to be studied further.
            pose_matrix_lhs = np.vstack([ pose[0,:], -pose[1,:], pose[2,:], pose[3,:] ])
            pose_matrix_lhs = np.matmul(pose_matrix_lhs, self._rotate_z_180)

            # Add to list of detections
            apriltag_detections.append(AprilTagDetection(tag_id = detection.tag_id, tag_size_cm = tag_size, pose_matrix = pose_matrix_lhs))

            # Reproject pose back into image space and render in cyan
            image_points = self._reproject(pose_matrix = pose, intrinsic_matrix = self._K_rgb, tag_size_cm = tag_size)
            for i in range(4):
                start, end = image_points[i], image_points[(i + 1) % 4]
                cv2.line(
                    img = color_frame,
                    pt1 = (int(start[0]), int(start[1])),
                    pt2 = (int(end[0]), int(end[1])),
                    color = (255, 255, 0),
                    thickness = 2
                )

        # Store most recent detections
        if len(self.detections) > 0:
            del self.detections
        self.detections = apriltag_detections
        if self.on_apriltags_detected is not None:
            self.on_apriltags_detected(device_id = device_id, detections = apriltag_detections)

    def _reproject(self, pose_matrix, intrinsic_matrix, tag_size_cm):
        object_points = tag_size_cm * np.vstack(
            [
                [-0.5, -0.5, 0],    # top-left
                [0.5, -0.5, 0],     # top-right
                [0.5, 0.5, 0],      # bottom-right
                [-0.5, 0.5, 0],     # bottom-left
            ]
        )
        world_points = []
        for i in range(4):
            object_point = object_points[i, :].reshape((3, 1))
            object_point = np.vstack([object_point, 1])  # turn into a 4x1 vector
            world_point = np.matmul(pose_matrix, object_point)[0:3, :]
            world_points.append(world_point)
        image_points = []
        for world_point in world_points:
            perspective = world_point / world_point[2, 0]  # divide by z
            image_point = np.matmul(intrinsic_matrix, perspective)[0:2, 0]
            image_points.append(image_point)
        return image_points
