using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Walls : MessageReceivingBehavior
{
    [SerializeField]
    [Tooltip("Prefabs to use for walls. Placed at y=0. The first one is the most perpendicular wall. The remaining prefabs are used as needed.")]
    private GameObject[] m_wallPrefabs;

    private IOAKDRegistrationProvider m_registration;
    private List<GameObject> m_walls = new List<GameObject>();

    public List<GameObject> DetectedWalls
    {
        get
        {
            return m_walls;
        }
    }

    private void Awake()
    {
        m_registration = FindObjectsOfType<MonoBehaviour>().Where((MonoBehaviour x) => x is IOAKDRegistrationProvider).First() as IOAKDRegistrationProvider;
        Debug.Assert(m_registration != null);
    }

    public override void OnAprilTagDetections(AprilTagDetection[] detections)
    {
        // Convert poses to HMD space and snap to gravity
        List<Matrix4x4> wallPoses = new List<Matrix4x4>();
        foreach (var detection in detections)
        {
            if (detection.tag_id != 1 && detection.tag_id != 2) // tag IDs 1 and 2 are for walls
            {
                continue;
            }
            Matrix4x4 hmdSpacePose = m_registration.ConvertAprilTagPoseToHMD(detection.pose_matrix);
            Quaternion gravityAlignedRotation = Quaternion.LookRotation(hmdSpacePose.Forward().XZProjected(), Vector3.up);
            Matrix4x4 wallPose = Matrix4x4.TRS(hmdSpacePose.Translation().XZProjected(), gravityAlignedRotation, Vector3.one);
            wallPoses.Add(wallPose);
        }

        for (int i = 0; i < wallPoses.Count; i++)
        {
            for (int j = 0; j < wallPoses.Count; j++)
            {
                Debug.LogFormat("Distance {0},{1} = {2}", i, j, Vector3.Distance(wallPoses[i].Translation(), wallPoses[j].Translation()));
            }
        }
        
        // Destroy existing walls if any exist
        RemoveWalls();

        // Sort detections beginning most perpendicular (smallest angle with normal to camera)
        Vector3 oakdForward = m_registration.OAKDCameraPose().Forward();
        wallPoses.Sort((a, b) => Vector3.Angle(a.Forward(), oakdForward).CompareTo(Vector3.Angle(b.Forward(), oakdForward)));
        
        // Create wall objects
        for (int i = 0; i < wallPoses.Count; i++)
        {
            GameObject prefab = m_wallPrefabs[i >= m_wallPrefabs.Length ? (m_wallPrefabs.Length - 1) : i];
            GameObject wall = Instantiate(prefab);
            wall.name = string.Format("Wall-{0}", i);
            wall.transform.parent = transform;
            wall.transform.position = wallPoses[i].Translation();
            wall.transform.rotation = wallPoses[i].Rotation();
            m_walls.Add(wall);
        }
    }

    private void RemoveWalls()
    {
        foreach (var wall in m_walls)
        {
            Destroy(wall);
        }
        m_walls.Clear();
    }
}
