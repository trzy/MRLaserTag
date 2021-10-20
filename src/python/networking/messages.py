from dataclasses import dataclass
import numpy as np
from typing import List


@dataclass
class HelloMessage:
    id: str
    app_type: str
    message: str

@dataclass
class PingMessage:
    timestamp: float

@dataclass
class AprilTagDetection:
    tag_id: int
    tag_size_cm: float
    pose_matrix: np.ndarray

@dataclass
class AprilTagDetectionsMessage:
  detections: List[AprilTagDetection]

@dataclass
class HeightmapMessage:
    heightmap: List[float]
    sample_count: List[float]
    width: int
    height: int
    cell_size: float

@dataclass
class Box:
    label: str
    pose_matrix: np.ndarray
    extents: np.ndarray

@dataclass
class BoxesMessage:
    boxes: List[Box]

@dataclass
class ModelBasedCalibrationControlMessage:
    startCalibration: bool

@dataclass
class HMDControllerPoseMessage:
    pose_matrix: np.ndarray

@dataclass
class VuforiaControllerPoseMessage:
    pose_matrix: np.ndarray

@dataclass
class ModelBasedRegistrationMessage:
    registration_matrix: np.ndarray
    oakd_controller_positions: List[np.ndarray]

@dataclass
class HMDAvatarPoseMessage:
    id: str
    head_pose: np.ndarray
    left_hand_pose: np.ndarray
    right_hand_pose: np.ndarray

@dataclass
class HMDLaserFiredMessage:
    id: str
    pose: np.ndarray
    fast_beam_mode: bool