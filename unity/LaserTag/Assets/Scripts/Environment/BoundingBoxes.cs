using System.Linq;
using UnityEngine;

public class BoundingBoxes : MessageReceivingBehavior
{
    [SerializeField]
    [Tooltip("Prefab to use for bounding boxes.")]
    private GameObject m_boundingBoxPrefab;

    private IOAKDRegistrationProvider m_registration;
    private Walls m_walls;

    private void Awake()
    {
        m_registration = FindObjectsOfType<MonoBehaviour>().Where((MonoBehaviour x) => x is IOAKDRegistrationProvider).First() as IOAKDRegistrationProvider;
        Debug.Assert(m_registration != null);
        m_walls = FindObjectsOfType<MonoBehaviour>().Where((MonoBehaviour x) => x is Walls).First() as Walls;
        Debug.Assert(m_walls != null);
    }

    public override void OnBoundingBoxes(Box[] boxes)
    {
#if UNITY_EDITOR
        RGBDViewer? rgbd = FindObjectOfType<RGBDViewer>();
        rgbd?.LoadAndRenderPointCloud();
        //return;
#endif
        Debug.LogFormat("Received {0} boxes", boxes.Length);
        foreach (Box box in boxes)
        {
            Vector3 center = box.pose_matrix.Translation();
            Transform wall = FindNearestWall(center);
            if (wall == null)
            {
                break;  // no walls
            }

            // Compute new extents of the box in the wall's coordinate system
            Vector3[] corners = GetBoxCorners(box);
            Vector3[] wallAlignedBoxExtents = ComputeBoxExtentsInWallFrame(corners, center, wall.transform);
            Vector3 minimum = wallAlignedBoxExtents[0];
            Vector3 maximum = wallAlignedBoxExtents[1];

            // Because walls are gravity aligned, we have a gravity aligned box
            // that we can now extend down to the floor. Simply compute the top
            // 4 points from the new extents, then the other 4 are projected to
            // y = 0. These points will be in the world coordinate system.
            corners[0] = center + minimum.x * wall.transform.right + maximum.y * wall.transform.up + minimum.z * wall.transform.forward;
            corners[1] = center + minimum.x * wall.transform.right + maximum.y * wall.transform.up + maximum.z * wall.transform.forward;
            corners[2] = center + maximum.x * wall.transform.right + maximum.y * wall.transform.up + minimum.z * wall.transform.forward;
            corners[3] = center + maximum.x * wall.transform.right + maximum.y * wall.transform.up + maximum.z * wall.transform.forward;
            corners[4] = corners[0].XZProjected();
            corners[5] = corners[1].XZProjected();
            corners[6] = corners[2].XZProjected();
            corners[7] = corners[3].XZProjected();
            Vector3 newCenter = 0.5f * new Vector3(corners[0].x + corners[3].x, corners[0].y + corners[7].y, corners[0].z + corners[3].z);

            GameObject boxObj = Instantiate(m_boundingBoxPrefab);
            boxObj.name = "BoundingBox-" + box.label;
            boxObj.transform.parent = transform;
            boxObj.transform.position = newCenter;
            boxObj.transform.rotation = wall.rotation;
            boxObj.transform.localScale = new Vector3(corners[3].x - corners[0].x, corners[3].y - 0, corners[3].z - corners[0].z);
        }
    }

    private Transform FindNearestWall(Vector3 position)
    {
        // We cheat a little and use the tag positions rather than trying to
        // actually locate the nearest points of each wall
        GameObject nearestWall = m_walls.DetectedWalls.OrderBy(wall => Vector3.Distance(wall.transform.position, position)).First();
        return nearestWall != null ? nearestWall.transform : null;
    }

    private Vector3[] GetBoxCorners(Box box)
    {
        Vector3 center = box.pose_matrix.Translation();
        Vector3 forward = box.pose_matrix.Forward();
        Vector3 up = box.pose_matrix.Up();
        Vector3 right = box.pose_matrix.Right();
        Vector3 extents = box.extents;
        return new Vector3[]
        {
            center + 0.5f * extents.x * right + 0.5f * extents.y * up + 0.5f * extents.z * forward,
            center + 0.5f * extents.x * right + 0.5f * extents.y * up - 0.5f * extents.z * forward,
            center + 0.5f * extents.x * right - 0.5f * extents.y * up + 0.5f * extents.z * forward,
            center + 0.5f * extents.x * right - 0.5f * extents.y * up - 0.5f * extents.z * forward,
            center - 0.5f * extents.x * right + 0.5f * extents.y * up + 0.5f * extents.z * forward,
            center - 0.5f * extents.x * right + 0.5f * extents.y * up - 0.5f * extents.z * forward,
            center - 0.5f * extents.x * right - 0.5f * extents.y * up + 0.5f * extents.z * forward,
            center - 0.5f * extents.x * right - 0.5f * extents.y * up - 0.5f * extents.z * forward
        };
    }

    private Vector3[] ComputeBoxExtentsInWallFrame(Vector3[] points, Vector3 center, Transform wall)
    {
        Vector3 minimum = Vector3.positiveInfinity;
        Vector3 maximum = Vector3.negativeInfinity;

        // Compute extents in the coordinate frame of the wall relative to box
        // center
        foreach (Vector3 point in points)
        {
            // Find distance of point from box center along each wall axis
            // using projection via dot product: P dot u = |P||u|cos(a),
            // |P|cos(a) = (P dot u)/|u| = P dot u because |u| = 1.
            float right = Vector3.Dot(point - center, wall.right);
            float up = Vector3.Dot(point - center, wall.up);
            float forward = Vector3.Dot(point - center, wall.forward);

            // Keep track of min/max along each axis
            minimum.x = Mathf.Min(minimum.x, right);
            minimum.y = Mathf.Min(minimum.y, up);
            minimum.z = Mathf.Min(minimum.z, forward);
            maximum.x = Mathf.Max(maximum.x, right);
            maximum.y = Mathf.Max(maximum.y, up);
            maximum.z = Mathf.Max(maximum.z, forward);
        }

        return new Vector3[] { minimum, maximum };
    }
}
