##
## Mixed Reality Laser Tag
## Copyright 2021 Bart Trzynadlowski
##
## registercams.py
##
## Camera registration process. Run in an environment with Python 3.8.8 and
## packages from requirements.txt.
##

import argparse
import asyncio
from collections import defaultdict
from collections import deque
from dataclasses import dataclass
import depthai as dai
from functools import partial
import numpy as np
import open3d as o3d
import sys
import time

from src.python.vision.camera import CameraTask
from src.python.vision.apriltags import AprilTagDetector


###############################################################################
# Camera Registration
#
# Match observations of a common AprilTag (id=0) calibration target between
# cameras and computes transform between cameras.
###############################################################################

@dataclass
class Observation:
    pose_matrix: np.ndarray
    device_id: str
    timestamp: float    # also serves as unique observation ID


@dataclass
class Correspondence:
    observation1: Observation
    observation2: Observation
    delta_time: float


observations_by_device_id = defaultdict(partial(deque, maxlen = 10))
correspondences = []

last_registration_time = -float("inf")

# Registration computed by taking single correspondence with observations most
# normal to camera
best_angular_deviation = float("inf")   # sum of angles of tag normals from camera normals
best_registration = None                # array of (device_id, matrix), where device_id is the camera frame we are transforming the other into


def _compute_angle_from_camera(pose_matrix):
    camera_forward = np.array([0, 0, 1])
    tag_forward = pose_matrix[0:3,2]
    tag_forward_magnitude = np.sqrt(np.sum(tag_forward * tag_forward))
    theta = np.arccos(np.dot(camera_forward, tag_forward) / tag_forward_magnitude)
    return theta


def _compute_registration_matrix(from_camera, to_camera):
    """
    Returns
    -------
    np.ndarray
        A 4x4 transformation matrix, R, that satisfies the equation:
        to_camera = R * from_camera
    """
    # M_to = R * M_from
    # M_to * M_from^-1 = R * M_from^-1 * M_from^-1
    # R = M_to * M_from^-1
    return np.matmul(to_camera, np.linalg.inv(from_camera))


def _update_registration_matrix():
    global best_registration
    global best_angular_deviation
    # See if we can find a correspondence with angular deviation less than the
    # current best
    did_update = False
    previous_best_angular_deviation = best_angular_deviation
    for correspondence in correspondences:
        angular_deviation = _compute_angle_from_camera(pose_matrix = correspondence.observation1.pose_matrix) + _compute_angle_from_camera(pose_matrix = correspondence.observation2.pose_matrix)
        if angular_deviation < best_angular_deviation or best_registration is None:
            registration_matrix1 = _compute_registration_matrix(from_camera = correspondence.observation2.pose_matrix, to_camera = correspondence.observation1.pose_matrix)
            registration_matrix2 = _compute_registration_matrix(from_camera = correspondence.observation1.pose_matrix, to_camera = correspondence.observation2.pose_matrix)
            best_registration = [ (correspondence.observation1.device_id, registration_matrix1), (correspondence.observation2.device_id, registration_matrix2) ]
            best_angular_deviation = angular_deviation
            did_update = True
    if did_update:
        print("Updated registration. Angular deviation change: %1.2f -> %1.2f" % (previous_best_angular_deviation, best_angular_deviation))
    else:
        print("Did not update registration")


def _correspondences_involve_a_common_observation(correspondence1, correspondence2):
    """
    Returns
    -------
    bool
        True if the two correspondences share at least one common observation.
        Observations are uniquely identified by their timestamp.
    """
    timestamps1 = [ correspondence1.observation1.timestamp, correspondence1.observation2.timestamp ]
    timestamps2 = [ correspondence2.observation1.timestamp, correspondence2.observation2.timestamp ]
    return not set(timestamps1).isdisjoint(timestamps2)


def on_apriltags_detected(device_id, detections):
    now = time.time()

    detections = [ detection for detection in detections if detection.tag_id == 0 ]
    if len(detections) > 1:
        print("Warning: Multiple calibration objects found by camera %s" % device_id)
    if len(detections) <= 0:
        return

    # Find calibration target (tag_id=0)
    tags = [ detection for detection in detections if detection.tag_id == 0 ]
    if len(tags) == 0:
        return
    tag = tags[0]

    # Store raw observation sample. No delta time yet.
    observations_by_device_id[device_id].append(Observation(pose_matrix = tag.pose_matrix, device_id = device_id, timestamp = now))

    # Perform O(N^2) search to find best corresponding pair of observations
    # (lowest time delta between cameras)
    best_delta_time = float("inf")
    best_correspondence = None
    for device_id1, observations1 in observations_by_device_id.items():
        for device_id2, observations2 in observations_by_device_id.items():
            if device_id1 == device_id2:
                continue
            for observation1 in observations1:
                for observation2 in observations2:
                    delta_time = abs(observation1.timestamp - observation2.timestamp)
                    if delta_time < best_delta_time or best_correspondence is None:
                        best_delta_time = delta_time
                        best_correspondence = Correspondence(observation1 = observation1, observation2 = observation2, delta_time = delta_time)
    if best_correspondence is None:
        return

    # If previous correspondences exists involving either of the observation
    # samples, remove them if the new one has a better delta time (because
    # being farther apart in time, they are worse correspondences).
    global correspondences
    global last_registration_time
    add_new_correspondence = False
    length_before = len(correspondences)
    correspondences = [ correspondence for correspondence in correspondences if not
        (_correspondences_involve_a_common_observation(correspondence1 = best_correspondence, correspondence2 = correspondence) and
         best_correspondence.delta_time < correspondence.delta_time)
    ]

    # Add the new correspondence if anything was removed
    if len(correspondences) == 0 or len(correspondences) < length_before:
        correspondences.append(best_correspondence)
        print("Added correspondence")

    # Periodically attempt to compute the registration and clear observations out
    if now - last_registration_time > 5:
        for device_id in observations_by_device_id:
            observations_by_device_id[device_id].clear()
        last_registration_time = now
        _update_registration_matrix()
        correspondences = []


###############################################################################
# Program Entry Point
###############################################################################

if __name__ == "__main__":
    parser = argparse.ArgumentParser("registercams")
    parser.add_argument("--window", metavar = "width,height", type = str, action = "store", default = "1024,576", help = "Display resolution of each camera feed")
    parser.add_argument("--file", metavar = "path", type = str, action = "store", help = "Write registration matrices to file")
    options = parser.parse_args()
    width, height = [ int(dimension) for dimension in options.window.split(",") ]

    cameras = []
    detectors = []  # keeps detector objects alive
    for device in dai.Device.getAllAvailableDevices():
        print("Found camera %s" % device.getMxId())
        camera = CameraTask(device_id = device.getMxId(), window_size = (width, height))
        cameras.append(camera)
        apriltag_detector = AprilTagDetector(K_rgb = camera.calibration["intrinsic_rgb"])
        apriltag_detector.on_apriltags_detected = on_apriltags_detected
        camera.add_rgb_subscriber(apriltag_detector)
        detectors.append(apriltag_detector)
    assert len(cameras) == 2, "Camera registration currently assumes exactly 2 cameras"

    loop = asyncio.get_event_loop()
    for camera in cameras:
        loop.create_task(camera.run(loop = loop))
    loop.run_forever()

    # Exited -- print registration matrices
    if best_registration != None:
        device_id1 = best_registration[0][0]
        device_id2 = best_registration[1][0]
        matrix12 = best_registration[1][1]
        matrix21 = best_registration[0][1]
        print("Registration Matrices")
        print("---------------------")
        print("%s -> %s:" % (device_id1, device_id2))
        print("m12=np.array([")
        print("  [ %f, %f, %f, %f ]," % (matrix12[0,0], matrix12[0,1], matrix12[0,2], matrix12[0,3]))
        print("  [ %f, %f, %f, %f ]," % (matrix12[1,0], matrix12[1,1], matrix12[1,2], matrix12[1,3]))
        print("  [ %f, %f, %f, %f ]," % (matrix12[2,0], matrix12[2,1], matrix12[2,2], matrix12[2,3]))
        print("  [ %f, %f, %f, %f ]" % (matrix12[3,0], matrix12[3,1], matrix12[3,2], matrix12[3,3]))
        print("])")
        print("%s -> %s:" % (device_id2, device_id1))
        print("m21=np.array([")
        print("  [ %f, %f, %f, %f ]," % (matrix21[0,0], matrix21[0,1], matrix21[0,2], matrix21[0,3]))
        print("  [ %f, %f, %f, %f ]," % (matrix21[1,0], matrix21[1,1], matrix21[1,2], matrix21[1,3]))
        print("  [ %f, %f, %f, %f ]," % (matrix21[2,0], matrix21[2,1], matrix21[2,2], matrix21[2,3]))
        print("  [ %f, %f, %f, %f ]" % (matrix21[3,0], matrix21[3,1], matrix21[3,2], matrix21[3,3]))
        print("])")

        if options.file is not None:
            with open(options.file, "w") as fp:
                fp.write("%s\n" % device_id1)
                fp.write("%s\n" % device_id2)
                fp.write("%f %f %f %f\n" % (matrix12[0,0], matrix12[0,1], matrix12[0,2], matrix12[0,3]))
                fp.write("%f %f %f %f\n" % (matrix12[1,0], matrix12[1,1], matrix12[1,2], matrix12[1,3]))
                fp.write("%f %f %f %f\n" % (matrix12[2,0], matrix12[2,1], matrix12[2,2], matrix12[2,3]))
                fp.write("%f %f %f %f\n" % (matrix12[3,0], matrix12[3,1], matrix12[3,2], matrix12[3,3]))
                fp.write("%s\n" % device_id2)
                fp.write("%s\n" % device_id1)
                fp.write("%f %f %f %f\n" % (matrix21[0,0], matrix21[0,1], matrix21[0,2], matrix21[0,3]))
                fp.write("%f %f %f %f\n" % (matrix21[1,0], matrix21[1,1], matrix21[1,2], matrix21[1,3]))
                fp.write("%f %f %f %f\n" % (matrix21[2,0], matrix21[2,1], matrix21[2,2], matrix21[2,3]))
                fp.write("%f %f %f %f\n" % (matrix21[3,0], matrix21[3,1], matrix21[3,2], matrix21[3,3]))
                print("Wrote registration matrices to %s" % options.file)
