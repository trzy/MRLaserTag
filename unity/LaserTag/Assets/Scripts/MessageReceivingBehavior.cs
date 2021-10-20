using UnityEngine;

public class MessageReceivingBehavior: MonoBehaviour
{
    public virtual void OnAprilTagDetections(AprilTagDetection[] detections)
    {
    }
    
    public virtual void OnBoundingBoxes(Box[] boxes)
    {
    }
    
    public virtual void OnModelBasedRegistration(Matrix4x4 registrationMatrix, Vector3[] oakdControllerPositions)
    {
    }

    public virtual void OnHMDAvatarPose(string clientID, Matrix4x4 headPose, Matrix4x4 leftHandPose, Matrix4x4 rightHandPose)
    {
    }

    public virtual void OnHMDLaserFired(string clientID, Matrix4x4 pose, bool fastBeamMode)
    {
    }
}