using System;
using UnityEngine;

[Serializable]
public struct HelloMessage
{
    public string id;
    public string app_type;
    public string message;

    public HelloMessage(string id, string app_type, string message)
    {
        this.id = id;
        this.app_type = app_type;
        this.message = message;
    }
}

[Serializable]
public struct PingMessage
{
    public float timestamp;

    public PingMessage(float timestamp)
    {
        this.timestamp = timestamp;
    }
}

[Serializable]
public struct AprilTagDetection
{
    public int tag_id;
    public float tag_size_cm;
    public Matrix4x4 pose_matrix;
}

[Serializable]
public struct AprilTagDetectionsMessage
{
    public AprilTagDetection[] detections;
}

[Serializable]
public struct HeightmapMessage
{
    public float[] heightmap;
    public float[] sample_count;
    public int width;
    public int height;
    public float cell_size;
}

[Serializable]
public struct Box
{
    public string label;
    public Matrix4x4 pose_matrix;
    public Vector3 extents;
}

[Serializable]
public struct BoxesMessage
{
    public Box[] boxes;
}

[Serializable]
public struct ModelBasedCalibrationControlMessage
{
    public bool startCalibration; // true to start a new calibration session, false ends current session
}

[Serializable]
public struct HMDControllerPoseMessage
{
    public Matrix4x4 pose_matrix;
}

[Serializable]
public struct ModelBasedRegistrationMessage
{
    public Matrix4x4 registration_matrix;
    public Vector3[] oakd_controller_positions;
}

[Serializable]
public struct HMDAvatarPoseMessage
{
    public string id;
    public Matrix4x4 head_pose;
    public Matrix4x4 left_hand_pose;
    public Matrix4x4 right_hand_pose;

    public HMDAvatarPoseMessage(string id, Matrix4x4 head_pose, Matrix4x4 left_hand_pose, Matrix4x4 right_hand_pose)
    {
        this.id = id;
        this.head_pose = head_pose;
        this.left_hand_pose = left_hand_pose;
        this.right_hand_pose = right_hand_pose;
    }
}

[Serializable]
public struct HMDLaserFiredMessage
{
    public string id;
    public Matrix4x4 pose;
    public bool fast_beam_mode;

    public HMDLaserFiredMessage(string id, Matrix4x4 pose, bool fast_beam_mode)
    {
        this.id = id;
        this.pose = pose;
        this.fast_beam_mode = fast_beam_mode;
    }
}