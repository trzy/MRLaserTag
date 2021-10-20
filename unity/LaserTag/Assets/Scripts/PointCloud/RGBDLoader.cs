using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public struct RGBDPoint
{
    public Vector3 position;
    public byte red;
    public byte green;
    public byte blue;
}

public static class RGBDLoader
{
    // Loads RGBD point cloud and converts to a y-up coordinate system
    public static List<RGBDPoint> LoadFile(string filepath)
    {
        string contents = System.IO.File.ReadAllText(filepath);
        string[] values = contents.Split().Where(value => value.Length > 0).ToArray();
        Debug.Assert(values.Length % 6 == 0);                                           // 6 elements per line: x, y, z, r, g, b 
        //Debug.Assert(values.Length % 6 == 0 && values.Length / 6 == (1280 * 720));      // entire frame was loaded
        List<RGBDPoint> rgbdPoints = new List<RGBDPoint>();
        for (int i = 0; i < values.Length; i += 6)
        {
            RGBDPoint point = new RGBDPoint();
            float x = float.Parse(values[i + 0]) / 100.0f;  // convert cm -> m
            float y = float.Parse(values[i + 1]) / 100.0f;  // no need to convert y-down -> y-up because this cloud should have been registered by the server
            float z = float.Parse(values[i + 2]) / 100.0f;
            point.position = new Vector3(x, y, z);
            point.red = (byte)float.Parse(values[i + 3]);
            point.green = (byte)float.Parse(values[i + 4]);
            point.blue = (byte)float.Parse(values[i + 5]);
            rgbdPoints.Add(point);
        }
        return rgbdPoints;
    }
}