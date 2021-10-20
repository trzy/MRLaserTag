using UnityEngine;

public static class Matrix4x4Extensions
{
    public static Vector3 Translation(this Matrix4x4 matrix)
    {
        Vector4 position = matrix.GetColumn(3);
        return new Vector3(position.x, position.y, position.z);
    }

    public static Vector3 Scale(this Matrix4x4 matrix)
    {
        Vector3 scale;
        scale.x = new Vector4(matrix.m00, matrix.m10, matrix.m20, matrix.m30).magnitude;
        scale.y = new Vector4(matrix.m01, matrix.m11, matrix.m21, matrix.m31).magnitude;
        scale.z = new Vector4(matrix.m02, matrix.m12, matrix.m22, matrix.m32).magnitude;
        return scale;
    }

    public static Quaternion Rotation(this Matrix4x4 matrix)
    {
        Vector3 forward;
        forward.x = matrix.m02;
        forward.y = matrix.m12;
        forward.z = matrix.m22;

        Vector3 up;
        up.x = matrix.m01;
        up.y = matrix.m11;
        up.z = matrix.m21;

        return Quaternion.LookRotation(forward, up);
    }

    public static Vector3 Forward(this Matrix4x4 matrix)
    {
        Vector4 column = matrix.GetColumn(2);
        return new Vector3(column.x, column.y, column.z).normalized;
    }

    public static Vector3 Up(this Matrix4x4 matrix)
    {
        Vector4 column = matrix.GetColumn(1);
        return new Vector3(column.x, column.y, column.z).normalized;
    }

    public static Vector3 Right(this Matrix4x4 matrix)
    {
        Vector4 column = matrix.GetColumn(0);
        return new Vector3(column.x, column.y, column.z).normalized;
    }
}