using UnityEngine;

public class DebugRegistration : MonoBehaviour
{
    [SerializeField]
    private GameObject m_hmdPointPrefab;

    [SerializeField]
    private GameObject m_oakdPointPrefab;

    [SerializeField]
    private GameObject m_oakdCameraPrefab;

    [SerializeField]
    private GameObject m_aprilTagPrefab;

    private Transform m_hmdPointRoot;
    private Transform m_oakdPointRoot;

    private void Awake()
    {
        GameObject hmdPointRoot = new GameObject("HMD Points");
        hmdPointRoot.transform.parent = transform;
        m_hmdPointRoot = hmdPointRoot.transform;
        GameObject oakdPointRoot = new GameObject("OAKD Points");
        oakdPointRoot.transform.parent = transform;
        m_oakdPointRoot = oakdPointRoot.transform;
    }

    private void Start()
    {
        Matrix4x4 aprilTagPose = Matrix4x4.identity;
        aprilTagPose.m00 = 7.89490816e-01f; aprilTagPose.m01 = 7.21710328e-02f; aprilTagPose.m02 = 6.09504384e-01f; aprilTagPose.m03 = -8.24772761e+01f;
        aprilTagPose.m10 = -1.60278765e-01f;  aprilTagPose.m11 = 9.82846715e-01f;  aprilTagPose.m12 = 9.12307637e-02f; aprilTagPose.m13 = -1.32284560e+01f;
        aprilTagPose.m20 = -5.92465163e-01f; aprilTagPose.m21 = -1.69716460e-01f;  aprilTagPose.m22 = 7.87515939e-01f;  aprilTagPose.m23 = 2.41529526e+02f;
        aprilTagPose.m30 = 0.00000000e+00f; aprilTagPose.m31 = 0.00000000e+00f;  aprilTagPose.m32 = 0.00000000e+00f; aprilTagPose.m33 = 1.00000000e+00f;

        Vector3[] hmdPoints = new Vector3[]
        {
            new Vector3(0.446641f,0.872977f,0.346183f),
            new Vector3(0.362805f,0.892503f,0.207608f),
            new Vector3(0.380290f,0.891049f,0.102070f),
            new Vector3(0.405263f,0.839662f,0.206630f),
            new Vector3(0.418736f,0.908935f,0.036843f),
            new Vector3(0.300231f,0.889149f,0.311767f),
            new Vector3(0.234275f,0.898181f,0.227993f),
        };
        Vector3[] oakdPoints = new Vector3[]
        {
            new Vector3(0.034331f,0.035914f,0.286719f),
            new Vector3(0.004126f,0.034775f,0.450841f),
            new Vector3(-0.076037f,0.038207f,0.520278f),
            new Vector3(-0.028091f,0.086499f,0.416095f),
            new Vector3(-0.147012f,0.024416f,0.540668f),
            new Vector3(0.118398f,0.031736f,0.414062f),
            new Vector3(0.110219f,0.035483f,0.519668f),
        };
        Matrix4x4 registrationMatrix = Matrix4x4.identity;
        registrationMatrix.m00 = -0.737032f;
        registrationMatrix.m01 = 0.197576f;
        registrationMatrix.m02 = -0.646334f;
        registrationMatrix.m03 = 0.647079f;
        registrationMatrix.m10 = 0.061300f;
        registrationMatrix.m11 = 0.971917f;
        registrationMatrix.m12 = 0.227201f;
        registrationMatrix.m13 = 0.742244f;
        registrationMatrix.m20 = 0.673072f;
        registrationMatrix.m21 = 0.127834f;
        registrationMatrix.m22 = -0.728445f;
        registrationMatrix.m23 = 0.526504f;
        registrationMatrix.m30 = 0.000000f;
        registrationMatrix.m31 = 0.000000f;
        registrationMatrix.m32 = 0.000000f;
        registrationMatrix.m33 = 1.000000f;

        // Create HMD points
        foreach (Vector3 hmdPoint in hmdPoints)
        {
            GameObject point = CreatePoint(m_hmdPointPrefab, m_hmdPointRoot);
            point.transform.position = hmdPoint;
        }

        // Create OAKD points and transform them using registration matrix
        foreach (Vector3 oakdPoint in oakdPoints)
        {
            GameObject point = CreatePoint(m_oakdPointPrefab, m_oakdPointRoot);
            Vector4 transformedPoint4 = registrationMatrix * new Vector4(oakdPoint.x, oakdPoint.y, oakdPoint.z, 1);
            point.transform.position = new Vector3(transformedPoint4.x, transformedPoint4.y, transformedPoint4.z);
        }

        // Create AprilTag object
        GameObject aprilTag = CreatePoint(m_aprilTagPrefab, transform);
        Matrix4x4 pose = registrationMatrix * FixAprilTagMatrix(aprilTagPose);
        aprilTag.transform.position = pose.Translation();
        aprilTag.transform.rotation = pose.Rotation();
        aprilTag.transform.localScale = new Vector3(pose.Scale().x * m_aprilTagPrefab.transform.localScale.x, pose.Scale().y * m_aprilTagPrefab.transform.localScale.y, pose.Scale().z * m_aprilTagPrefab.transform.localScale.z);

        // Create OAKD camera object
        GameObject camera = CreatePoint(m_oakdCameraPrefab, transform);
        Vector4 oakdOrigin4 = registrationMatrix * new Vector4(0, 0, 0, 1);
        camera.transform.position = new Vector3(oakdOrigin4.x, oakdOrigin4.y, oakdOrigin4.z);
    }

    // AprilTags are detected in a forward=+z coordinate system whereas Vuforia is forward=-z.
    // The Python server flips the z component of the Vuforia positions before computing the
    // registration matrix, therefore all OAKD observations -- AprilTags and Vuforia controller
    // poses -- are in the same coordinate system. The only adjustment required of AprilTags is
    // a size conversion from centimeters to meters.
    private Matrix4x4 FixAprilTagMatrix(Matrix4x4 matrix)
    {
        // Position from cm -> m
        matrix.m03 *= 0.01f;
        matrix.m13 *= 0.01f;
        matrix.m23 *= 0.01f;
        return matrix;
    }

    private GameObject CreatePoint(GameObject prefab, Transform parent)
    {
        GameObject point = Instantiate(prefab);
        point.transform.parent = parent;
        return point;
    }
}
