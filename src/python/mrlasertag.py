##
## Mixed Reality Laser Tag
## Copyright 2021 Bart Trzynadlowski
##
## mrlasertag.py
##
## Main laser tag server process. Run in an environment with Python 3.8.8 and
## packages from requirements.txt.
##

#
# Coordinate Systems and HMD-Camera Registration
# ----------------------------------------------
#
# y                               y
# ^                               ^
# |                               |
# |      z                        |
# |     /|                        |
# |    /                          |
# |   /                           +---------------------> x
# |  /                           /
# | /                           /
# |/                          |/
# +---------------------> x   z
#      Left-Handed                       Right-Handed
#
# HMD uses LHS whereas the camera is an RHS system with y-down. Taking the
# above standard diagram and rotating 180 degrees about x produces:
#
#                        z
#                       /|
#                      /
#                     /
#                    /
#                   /
#                  /
#                 +---------------------> x
#                 |
#                 |
#                 |
#                 |
#                 |
#                 |
#                 |
#                 |
#                 V
#                 y
#
# A right-handed coordinate system with:
#
#   +x: Right
#   +y: Down
#   +z: Away from camera (for depth values)
#
# Vuforia unfortunately uses a left-handed coordinate system:
#
#   +x: Right
#   +y: Down
#   -z: Away from camera
#
# To compute the registration matrix between the camera and HMD, we sample
# controller poses on the HMD and using Vuforia. A point cloud registration
# solver is run to compute the transform matrix that converts the camera
# observations to their equivalent HMD points. However, because of the mis-
# matching "handedness" of the coordinate systems, the solver does not
# produce a very good transformation. Enabling scale as a degree of freedom
# has not been attemted but most likely, the problem is that the solver
# assumes some constraints during its search that are incompatible with the
# manipulation needed to switch coordinate systems.
#
# The solution for Vuforia observations is simple: both z and y are inverted
# for each controller translation. For point clouds obtained by the depth
# camera, only y must be inverted. AprilTags should involve a simple inversion
# of the y axis but an additional rotation is required and described in
# apriltags.py.
#
# With both HMD- and camera-based controller positions in right-handed frames,
# point cloud registration produces a very accurate transformation matrix.
#
#
# TODO
# ----
# - Perform AprilTag distance scale adjustment on server side.
# - More robust filtering of correspondences
#   - What about the RANSAC parameter?
#   - Can we use distance comparisons between pairs of
#     points to filter out bad correspondences such as
#     duplicates appearing in one of the coordinate spaces?
#   - How small can we make the correspondence time threshold?
#   - These are all low-priority because registration is generally
#     very good.
#

import argparse
import asyncio
from collections import deque
from copy import deepcopy
import depthai as dai
from enum import Enum
import numpy as np
import open3d as o3d
import sys
import time

from src.python.vision.camera import CameraTask
from src.python.vision.apriltags import AprilTagDetector
from src.python.vision.height_map import HeightMapGenerator
from src.python.vision.object_detector import ObjectDetector
from src.python.vision.vuforia import VuforiaForwarder
from src.python.networking.tcp import Server
from src.python.networking.messages import *
from src.python.networking.message_handling import handler
from src.python.networking.message_handling import MessageHandler
from src.python.networking.serialization import LaserTagJSONEncoder


###############################################################################
# HMD-Camera Registration
###############################################################################

def compute_registration_matrix(point_correspondences):
    # Reference point cloud: HMD space
    hmd_points = np.array([ pair[0] for pair in point_correspondences ])
    hmd_cloud = o3d.geometry.PointCloud()
    hmd_cloud.points = o3d.utility.Vector3dVector(hmd_points)

    # Source point cloud: OAKD/Vuforia space
    oakd_points = np.array([ pair[1] for pair in point_correspondences ])
    oakd_cloud = o3d.geometry.PointCloud()
    oakd_cloud.points = o3d.utility.Vector3dVector(oakd_points)

    # Correspondences: 1:1 between the two point arrays
    num_points = len(point_correspondences)
    correspondences = np.vstack([ np.arange(num_points), np.arange(num_points) ]).T
    correspondences = o3d.utility.Vector2iVector(correspondences)

    # Perform correspondence-based fit
    result = o3d.pipelines.registration.registration_ransac_based_on_correspondence(source = oakd_cloud, target = hmd_cloud, corres = correspondences, max_correspondence_distance = 10000, ransac_n = num_points)
    registration_transform_matrix = result.transformation

    # Test results by printing per-point error
    oakd_points = np.hstack([ oakd_points, np.ones([num_points, 1]) ])
    registered_points = np.dot(registration_transform_matrix, oakd_points.T).T
    print("Error (reference_points - registered_points):")
    for i in range(registered_points.shape[0]):
        print("  %s" % (hmd_points[i] - registered_points[i][0:3]))

    return registration_transform_matrix


###############################################################################
# Server Logic
#
# Called from server task.
###############################################################################

class MessageProcessor(MessageHandler):
    class _RegistrationSession:
        def __init__(self):
            self.oakd_controller_positions = deque(maxlen = 60) # sliding window of last 60 samples (1 sec of data at 60 Hz, 2 sec at 30 Hz)
            self.hmd_oakd_controller_correspondences = []       # corresponding pairs of HMD-space and OAKD-space controller positions

        def find_corresponding_oakd_controller_position(self, timestamp):
            """
            Scans the list of OAKD controller positions and returns the index of
            the nearest match in terms of time. Returns None if there is no match
            or if the nearest match is
            """
            max_delta_seconds = 0.1
            ascending_by_delta_time = sorted(self.oakd_controller_positions, key = lambda sample: abs(sample[0] - timestamp))
            if len(ascending_by_delta_time) > 0:
                best_delta_seconds = abs(timestamp - ascending_by_delta_time[0][0])
                if best_delta_seconds <= max_delta_seconds:
                    return ascending_by_delta_time[0][1]
            return None

    def __init__(self, cameras, apriltag_detectors, object_detector, options):
        super().__init__()

        assert len(cameras) >= 1
        self._cameras = cameras # first camera is main camera
        self._vuforia_forwarder = VuforiaForwarder()
        self._apriltag_detectors = apriltag_detectors
        self._object_detector = object_detector
        self._video_path = options.video

        # Clients
        self._sessions = set()
        self._client_info_by_session = {}
        self._pending_spectators = []   # spectators can join any time and are made pending until at least one HMD is registered

        # HMD/camera registration session
        self._registration_session_by_network_session = {}

        # Registration matrix per client
        self._spectator_registration_matrix = None  # spectators are special and can use any HMD's coordinate system, so we will use the first HMD that comes along
        self._registration_matrix_by_session = {}
        self._transform_matrix_by_src_dest = {}     # (session1,session2) -> registration_matrix

    def _recompute_all_transforms_between_sessions(self):
        # Recompute transform matrices between all clients
        self._transform_matrix_by_src_dest.clear()
        for src_session, src_matrix in self._registration_matrix_by_session.items():
            for dest_session, dest_matrix in self._registration_matrix_by_session.items():
                if src_session == dest_session:
                    src_to_dest = np.eye(4)
                else:
                    src_to_oakd = np.linalg.inv(src_matrix)
                    oakd_to_dest = dest_matrix
                    src_to_dest = np.matmul(oakd_to_dest, src_to_oakd)
                self._transform_matrix_by_src_dest[(src_session, dest_session)] = src_to_dest

    def _send_environment_snapshot(self, sessions, registration_matrix, oakd_controller_positions):
        """
        Sends snapshot of environment data to specified clients. This includes
        a registration matrix appropriate for the client, AprilTags, and
        detected object boxes.
        """
        # Send registration message
        msg = ModelBasedRegistrationMessage(registration_matrix = registration_matrix, oakd_controller_positions = oakd_controller_positions)
        for session in sessions:
            session.send(msg)

        # Send AprilTag detections now that HMD client is registered
        self._send_apriltags(sessions = sessions)

        # Detect object bounding boxes
        boxes = self._object_detector.get_transformed_boxes(oakd_to_hmd_matrix = registration_matrix)
        self._send_bounding_boxes(sessions = sessions, boxes = boxes)

    @staticmethod
    def _overlaps_existing_detections(existing_detections, detection):
        threshold = 1.0 * 100   # detections cannot be closer than this many cm
        position1 = detection.pose_matrix[0:3,3]
        for existing in existing_detections:
            position2 = existing.pose_matrix[0:3,3]
            distance = np.sqrt(np.sum((position2 - position1) * (position2 - position1)))
            if distance < threshold:
                return True
        return False

    def _send_apriltags(self, sessions):
        all_detections = []
        for camera, detector in zip(self._cameras, self._apriltag_detectors):
            # Transform all to main camera space
            detections = deepcopy(detector.detections)
            for i in range(len(detections)):
                detections[i].pose_matrix = np.matmul(camera.main_camera_registration_matrix, detections[i].pose_matrix)

            # Add detections to complete list only if they are not too close to
            # an existing detection
            for detection in detections:
                if not self._overlaps_existing_detections(existing_detections = all_detections, detection = detection):
                    all_detections.append(detection)

        # Send to clients
        msg = AprilTagDetectionsMessage(detections = all_detections)
        if len(msg.detections) > 0:
            for session in sessions:
                session.send(msg)

    def _send_bounding_boxes(self, sessions, boxes):
        msg = BoxesMessage(boxes = boxes)
        print("Boxes: %d" % len(msg.boxes))
        for session in sessions:
            session.send(msg)

    def on_connect(self, session):
        self._sessions.add(session)

    def on_disconnect(self, session):
        if session in self._sessions:
            self._sessions.remove(session)
        if session in self._client_info_by_session:
            del self._client_info_by_session[session]
        if session in self._pending_spectators:
            self._pending_spectators.remove(session)
        if session in self._registration_matrix_by_session:
            del self._registration_matrix_by_session[session]
        self._recompute_all_transforms_between_sessions()

    @handler(HelloMessage)
    def handle_HelloMessage(self, session, msg, timestamp):
        print("Got hello message: (%s) %s" % (msg.app_type, msg.message))
        self._client_info_by_session[session] = msg # hello message contains client information

        if msg.app_type == "Spectator":
            if self._spectator_registration_matrix is None:
                # No HMDs have been registered to the camera yet; need to wait
                self._pending_spectators.append(session)
            else:
                # Send environment snapshot
                self._registration_matrix_by_session[session] = self._spectator_registration_matrix
                self._send_environment_snapshot(sessions = [ session ], registration_matrix = self._spectator_registration_matrix, oakd_controller_positions = [])
                self._recompute_all_transforms_between_sessions()

    @handler(PingMessage)
    def handle_PingMessage(self, session, msg, timestamp):
        pass

    @handler(VuforiaControllerPoseMessage)
    def handle_VuforiaControllerPoseMessage(self, session, msg, timestamp):
        position = msg.pose_matrix[0:3,3]
        position[2] *= -1.0 # flip z so that positive is away from camera (RHS -> LHS)
        position[1] *= -1.0 # flip y (RHS -> LHS)
        position_sample = (timestamp, position)

        # This is a sample from Vuforia -- we don't know exactly which client
        # it corresponds to, so add it to all (only one session should be in
        # progress anyway)
        for _, registration in self._registration_session_by_network_session.items():
            registration.oakd_controller_positions.append(position_sample)

    @handler(ModelBasedCalibrationControlMessage)
    def handle_ModelBasedCalibrationControlMessage(self, session, msg, timestamp):
        if msg.startCalibration:
            print("Model-based calibration started")
            self._registration_session_by_network_session[session] = MessageProcessor._RegistrationSession()
            self._cameras[0].add_rgb_subscriber(self._vuforia_forwarder)    # Vuforia app needs main camera
        else:
            # Get existing session
            if not session in self._registration_session_by_network_session:
                return
            registration = self._registration_session_by_network_session[session]
            print("Model-based calibration ended")

            self._cameras[0].remove_rgb_subscriber(self._vuforia_forwarder)  # camera no longer needed

            # Debug: print out all correspondences
            print("  hmd_points = np.array([")
            for correspondence in registration.hmd_oakd_controller_correspondences:
                print("    [ %f, %f, %f ]," % (correspondence[0][0], correspondence[0][1], correspondence[0][2]))
            print("  ])")
            print("  oakd_points = np.array([")
            for correspondence in registration.hmd_oakd_controller_correspondences:
                print("    [ %f, %f, %f ]," % (correspondence[1][0], correspondence[1][1], correspondence[1][2]))
            print("  ])")

            # Compute registration matrix
            registration_matrix = compute_registration_matrix(point_correspondences = registration.hmd_oakd_controller_correspondences)

            print("---")
            print("Vector3[] hmdPoints = new Vector3[]")
            print("{")
            for correspondence in registration.hmd_oakd_controller_correspondences:
                print("    new Vector3(%ff,%ff,%ff)," % (correspondence[0][0], correspondence[0][1], correspondence[0][2]))
            print("};")
            print("Vector3[] oakdPoints = new Vector3[]")
            print("{")
            for correspondence in registration.hmd_oakd_controller_correspondences:
                print("    new Vector3(%ff,%ff,%ff)," % (correspondence[1][0], correspondence[1][1], correspondence[1][2]))
            print("};")
            print("Matrix4x4 registrationMatrix = Matrix4x4.identity;")
            for row in range(4):
                for col in range(4):
                    print("registrationMatrix.m%d%d = %ff;" % (row, col, registration_matrix[row, col]))

            # End registration session
            del self._registration_session_by_network_session[session]

            # If we don't yet have a registration matrix for spectators, use this one
            if self._spectator_registration_matrix is None:
                self._spectator_registration_matrix = registration_matrix

            # Send environment data to HMD
            self._registration_matrix_by_session[session] = registration_matrix
            oakd_controller_positions = [ pair[1] for pair in registration.hmd_oakd_controller_correspondences ]
            self._send_environment_snapshot(sessions = [ session ], registration_matrix = registration_matrix, oakd_controller_positions = oakd_controller_positions)

            # Handle pending spectators
            for spectator_session in self._pending_spectators:
                self._registration_matrix_by_session[spectator_session] = self._spectator_registration_matrix
            self._send_environment_snapshot(sessions = self._pending_spectators, registration_matrix = self._spectator_registration_matrix, oakd_controller_positions = [])
            self._pending_spectators = []

            # Recompute transform matrices between all clients
            self._recompute_all_transforms_between_sessions()

            # Start video recording on main camera
            if self._video_path:
                self._cameras[0].start_video_recording(video_path = self._video_path)

    @handler(HMDControllerPoseMessage)
    def handle_HMDControllerPoseMessage(self, session, msg, timestamp):
        if session in self._registration_session_by_network_session:
            registration = self._registration_session_by_network_session[session]
            print("HMD controller position:", msg.pose_matrix[0:3,3])
            oakd_position = registration.find_corresponding_oakd_controller_position(timestamp = timestamp)
            if oakd_position is not None:
                hmd_position = msg.pose_matrix[0:3,3]
                registration.hmd_oakd_controller_correspondences.append((hmd_position, oakd_position))
                registration.oakd_controller_positions.clear()
                print("  Found correspondence")
            else:
                print("  No correspondence found")

    @handler(HMDAvatarPoseMessage)
    def handle_HMDAvatarPoseMessage(self, session, msg, timestamp):
        # Forward to all other clients
        for peer_session in self._sessions:
            conversion = (session, peer_session)
            if peer_session != session and conversion in self._transform_matrix_by_src_dest:
                transform_matrix = self._transform_matrix_by_src_dest[conversion]
                converted_msg = HMDAvatarPoseMessage(
                    id = msg.id,
                    head_pose = np.matmul(transform_matrix, msg.head_pose),
                    left_hand_pose = np.matmul(transform_matrix, msg.left_hand_pose),
                    right_hand_pose = np.matmul(transform_matrix, msg.right_hand_pose)
                )
                peer_session.send(converted_msg)

    @handler(HMDLaserFiredMessage)
    def handle_HMDLaserFiredMessage(self, session, msg, timestamp):
        # Forward to all other clients
        for peer_session in self._sessions:
            conversion = (session, peer_session)
            if peer_session != session and conversion in self._transform_matrix_by_src_dest:
                transform_matrix = self._transform_matrix_by_src_dest[conversion]
                converted_msg = HMDLaserFiredMessage(
                    id = msg.id,
                    pose = np.matmul(transform_matrix, msg.pose),
                    fast_beam_mode = msg.fast_beam_mode
                )
                peer_session.send(converted_msg)


###############################################################################
# Cameras
#
# The main camera coordinate system is registered to the HMD and is used
# exclusively for object detection. Optional satellite cameras, each registered
# to the main camera, may be connected and any AprilTags they detect will be
# transformed to main camera space.
###############################################################################

def load_camera_registration_matrices(path):
    if path is None:
        return {}
    registration_matrix_by_device_ids = {}
    with open(path, "r") as fp:
        lines = fp.readlines()
    i = 0
    while i < len(lines):
        # Registration matrix from device 1 -> frame of device 2
        device_id1 = lines[i + 0].strip()
        device_id2 = lines[i + 1].strip()
        matrix12 = np.eye(4)
        for row in range(4):
            matrix12[row,:] = [ float(value.strip()) for value in lines[i + 2 + row].split() ]
        registration_matrix_by_device_ids[(device_id1, device_id2)] = matrix12
        i += 6
    print("Loaded %d matrices from %s" % (len(registration_matrix_by_device_ids), path))
    return registration_matrix_by_device_ids


def create_cameras(options):
    satellite_ids = []
    if not options.main_camera:
        for device in dai.Device.getAllAvailableDevices():
            is_main_camera = device.getMxId() == options.device
            print("Found %s camera %s" % ("main" if is_main_camera else "satellite", device.getMxId()))
            if not is_main_camera:
                satellite_ids.append(device.getMxId())
    registration_matrix_by_device_ids = load_camera_registration_matrices(path = options.registration) if len(satellite_ids) > 0 else {}
    window_size = (1920, 1080) if len(satellite_ids) == 0 else (1024, 576)  # reduced window size when using multiple cameras
    main_camera = CameraTask(device_id = options.device, window_size = window_size, show_fps = options.show_fps)
    print("Camera FOV = %f" % main_camera.calibration["horizontal_field_of_view_rgb"])
    satellite_cameras = []
    for satellite_id in satellite_ids:
        key = (satellite_id, options.device)    # want matrix that transforms this satellite camera -> main
        matrix = registration_matrix_by_device_ids[key] if key in registration_matrix_by_device_ids else np.eye(4)
        satellite_cameras.append(CameraTask(device_id = satellite_id, main_camera_registration_matrix = matrix, window_size = window_size))
    return main_camera, satellite_cameras


###############################################################################
# Program Entry Point
###############################################################################

if __name__ == "__main__":
    parser = argparse.ArgumentParser("mrlasertag")
    parser.add_argument("--device", metavar = "id", type = str, action = "store", default = "14442C10512EC0D200", help = "Device ID of main OAKD camera to use")
    parser.add_argument("--main-camera", action = "store_true", help = "Use only the main camera")
    parser.add_argument("--registration", metavar = "path", type = str, default = "assets/registration.txt", action = "store", help = "File to load camera registration matrices from")
    parser.add_argument("--video", metavar = "path", type = str, action = "store", help = "Write main camera video stream to file")
    parser.add_argument("--show-fps", action = "store_true", help = "Show frame rate in RGB feeds")
    options = parser.parse_args()

    print("Hello")

    main_camera, satellite_cameras = create_cameras(options = options)
    cameras = [ main_camera ] + satellite_cameras
    assert cameras[0].device_id == options.device, "First camera must be main camera"
    apriltag_detectors = [ AprilTagDetector(K_rgb = camera.calibration["intrinsic_rgb"]) for camera in cameras ]
    object_detector = ObjectDetector(calibration = main_camera.calibration)
    #main_camera.add_rgb_subscriber(object_detector)
    #main_camera.add_depth_subscriber(object_detector)
    processor = MessageProcessor(cameras = cameras, apriltag_detectors = apriltag_detectors, object_detector = object_detector, options = options)

    loop = asyncio.get_event_loop()
    server = Server(message_handler = processor)
    loop.create_task(server.run_server())
    for camera, detector in zip(cameras, apriltag_detectors):
        loop.create_task(camera.run(loop = loop))
        loop.create_task(detector.run(camera_task = camera))
    loop.create_task(object_detector.run(camera_task = main_camera))    # object detector runs only on main camera
    loop.run_forever()