using UnityEngine;

public interface IOAKDRegistrationProvider
{
    public void StartRegistration();
    public bool IsRegistrationFinished
    {
        get;
    }
    public Matrix4x4 ConvertAprilTagPoseToHMD(Matrix4x4 pose);
    public Matrix4x4 ConvertPoseToHMD(Matrix4x4 pose);
    public Matrix4x4 OAKDCameraPose();
}