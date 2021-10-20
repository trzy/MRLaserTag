using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;

public class RGBDViewer : MonoBehaviour
{
    [Tooltip("Directory containing point cloud file (points.txt) and AprilTag pose file (poses.txt) produced by apriltag_rgbd_capture")]
    [SerializeField]
    private string m_directory = "c:\\projects\\MRLaserTag";

    [Tooltip("Size (meters) that the cube representing each point will be rendered at")]
    [SerializeField]
    private float m_pointSize = 0.0075f;

    public void LoadAndRenderPointCloud()
    {
        GameObject pointCloudContainer = new GameObject("PointCloud");
        pointCloudContainer.transform.parent = transform;
        LoadPointCloud(pointCloudContainer);
    }

    private void LoadPointCloud(GameObject root)
    {
        string filepath = Path.Combine(m_directory, "points.txt");
        List<RGBDPoint> points = RGBDLoader.LoadFile(filepath);
        Debug.LogFormat("Loaded {0} points from {1}", points.Count, filepath);

        Vector3[] unitCubeVerts = new Vector3[]
        {
            new Vector3(-0.5f, 0.5f, -0.5f),  // 0 front top left
            new Vector3(0.5f, 0.5f, -0.5f),   // 1 front top right
            new Vector3(0.5f, -0.5f, -0.5f),  // 2 front bottom right
            new Vector3(-0.5f, -0.5f, -0.5f), // 3 front bottom left
            new Vector3(-0.5f, 0.5f, 0.5f),   // 4 back top left
            new Vector3(0.5f, 0.5f, 0.5f),    // 5 back top right
            new Vector3(0.5f, -0.5f, 0.5f),   // 6 back bottom right
            new Vector3(-0.5f, -0.5f, 0.5f),  // 7 back bottom left
        };

        int[] unitCubeTriangles = new int[]
        {
            3, 0, 2, 2, 0, 1, // front face
            6, 5, 4, 4, 7, 6, // back face
            0, 4, 5, 5, 1, 0, // top face
            3, 2, 6, 6, 7, 3, // bottom face
            4, 0, 3, 3, 7, 4, // left face
            1, 5, 6, 6, 2, 1  // right face
        };

        // Meshes are restricted to 65536 vertices, so we must break up the points
        int maxPointsPerMesh = 65536 / (4 * (unitCubeTriangles.Length / 3));//unitCubeVerts.Length;
        List<List<RGBDPoint>> pointsPerMesh = new List<List<RGBDPoint>>();
        for (int i = 0; i < points.Count;)
        {
            List<RGBDPoint> pointsThisMesh = new List<RGBDPoint>();

            int j = 0;
            for (; j < maxPointsPerMesh && (i + j) < points.Count; j++)
            {
                if (points[i + j].position.z < 20)  // arbitrary distance threshold
                {
                    pointsThisMesh.Add(points[i + j]);
                }
            }

            i += j;
            pointsPerMesh.Add(pointsThisMesh);
        }

        // Construct a mesh for each subset of points
        foreach (List<RGBDPoint> subpoints in pointsPerMesh)
        {
            BuildMesh(root, subpoints, unitCubeVerts, unitCubeTriangles);
        }
    }

    private void BuildMesh(GameObject root, List<RGBDPoint> points, Vector3[] unitCubeVerts, int[] unitCubeTriangles)
    {
        // Create a new child GameObject with mesh
        GameObject go = new GameObject();
        go.transform.parent = root.transform;
        MeshFilter meshFilter = go.AddComponent<MeshFilter>();
        Mesh mesh = meshFilter.mesh;
        MeshRenderer renderer = go.AddComponent<MeshRenderer>();
        renderer.material = new Material(Shader.Find("Unlit/RGBDPointCloud"));

        // Construct the mesh
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();
        List<Color> colors = new List<Color>();

        foreach (RGBDPoint point in points)
        {
            Vector3[] pointVerts = unitCubeVerts.Select(vert => m_pointSize * vert + point.position).ToArray(); // scale and position the point
            int[] pointTriangles = unitCubeTriangles.Select(index => index + vertices.Count).ToArray();         // append triangles
            vertices.AddRange(pointVerts);
            triangles.AddRange(pointTriangles);
            Color color = new Color32(point.red, point.green, point.blue, 255);
            for (int i = 0; i < unitCubeVerts.Length; i++)
            {
                colors.Add(color);
            }
        }

        Debug.LogFormat("Build mesh: {0} vertices, {1} triangles", vertices.Count, triangles.Count);

        mesh.vertices = vertices.ToArray();
        mesh.triangles = triangles.ToArray();
        mesh.colors = colors.ToArray();
        mesh.RecalculateBounds();
    }
}
