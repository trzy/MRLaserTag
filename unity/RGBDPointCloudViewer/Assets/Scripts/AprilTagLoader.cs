using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public struct AprilTag
{
    public Matrix4x4 pose;
    public float size;
}

public static class AprilTagLoader
{
    // Loads tag poses and converts them to a y-up coordinate system
    public static List<AprilTag> LoadFile(string filepath)
    {
        string contents = System.IO.File.ReadAllText(filepath);
        string[] values = contents.Split().Where(value => value.Length > 0).ToArray();
        int numTags = int.Parse(values[0]);
        Debug.Assert(values.Length == 1 + numTags * (4 * 4 + 1));
        List<AprilTag> aprilTags = new List<AprilTag>(numTags);
        for (int i = 1; i < values.Length; i += (4 * 4 + 1))
        {
            AprilTag aprilTag = new AprilTag();
            aprilTag.size = float.Parse(values[i + 0]) / 100.0f;
            aprilTag.pose = new Matrix4x4();
            aprilTag.pose.m00 = float.Parse(values[i + 1]);
            aprilTag.pose.m01 = float.Parse(values[i + 2]);
            aprilTag.pose.m02 = float.Parse(values[i + 3]);
            aprilTag.pose.m03 = float.Parse(values[i + 4]) / 100.0f;  // translation component from cm -> m
            aprilTag.pose.m10 = -float.Parse(values[i + 5]);          // invert all y components (the entire row 1)
            aprilTag.pose.m11 = -float.Parse(values[i + 6]);
            aprilTag.pose.m12 = -float.Parse(values[i + 7]);
            aprilTag.pose.m13 = -float.Parse(values[i + 8]) / 100.0f;
            aprilTag.pose.m20 = float.Parse(values[i + 9]);
            aprilTag.pose.m21 = float.Parse(values[i + 10]);
            aprilTag.pose.m22 = float.Parse(values[i + 11]);
            aprilTag.pose.m23 = float.Parse(values[i + 12]) / 100.0f;
            aprilTag.pose.m30 = float.Parse(values[i + 13]);
            aprilTag.pose.m31 = float.Parse(values[i + 14]);
            aprilTag.pose.m32 = float.Parse(values[i + 15]);
            aprilTag.pose.m33 = float.Parse(values[i + 16]);
            aprilTags.Add(aprilTag);
        }
        return aprilTags;
    }
}