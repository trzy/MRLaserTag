import asyncio
import cv2
import numpy as np


# For now, we use HMD coordinate system x and z axes as our grid but eventually we should pick a
# wall (one of the AprilTags), constrained to be gravity aligned, and use that as our coord system
class HeightMapGenerator:
    def __init__(self):
        self.on_heightmap_generated = None  # function to call when heightmap is produced

        self.oakd_to_hmd_matrix = np.eye(4)                         # conversion from OAKD to HMD world space
        self._cell_size = 20e-2                                     # side length of each cell in height map
        self._map_size = 20                                         # width of height map along either dimension
        self._grid_cells = int(self._map_size / self._cell_size)    # number of grid cells along each dimension (x and z)
        self._heightmap = np.zeros((self._grid_cells, self._grid_cells))    # max height within each cell

        # Temporary test matrices
        self.oakd_to_hmd_matrix[0,0] = 0.563389
        self.oakd_to_hmd_matrix[0,1] = 0.759209
        self.oakd_to_hmd_matrix[0,2] = 0.325875
        self.oakd_to_hmd_matrix[0,3] = 0.464947
        self.oakd_to_hmd_matrix[1,0] = 0.810693
        self.oakd_to_hmd_matrix[1,1] = -0.584042
        self.oakd_to_hmd_matrix[1,2] = -0.040889
        self.oakd_to_hmd_matrix[1,3] = 0.926027
        self.oakd_to_hmd_matrix[2,0] = 0.159282
        self.oakd_to_hmd_matrix[2,1] = 0.287220
        self.oakd_to_hmd_matrix[2,2] = -0.944528
        self.oakd_to_hmd_matrix[2,3] = 0.708595
        self.oakd_to_hmd_matrix[3,0] = 0.000000
        self.oakd_to_hmd_matrix[3,1] = 0.000000
        self.oakd_to_hmd_matrix[3,2] = 0.000000
        self.oakd_to_hmd_matrix[3,3] = 1.000000

        #self.oakd_to_hmd_matrix = np.eye(4)

        # Temporary AprilTag pose matrix (in camera space)
        # AprilTags have +z pointing into wall, y up
        self.apriltag_pose_matrix = np.array([
            [ 9.96134761e-01, -4.72117172e-02, -7.40715366e-02, -3.70977164e+01 ],
            [ 6.12671328e-02,  9.77723542e-01,  2.00756107e-01, -5.67753898e-02 ],
            [ 6.29434446e-02, -2.04518287e-01,  9.76836933e-01,  3.75462610e+02 ],
            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00 ]
        ])
        self.apriltag_pose_matrix[0,3] *= 0.01  # cm -> m
        self.apriltag_pose_matrix[1,3] *= 0.01
        self.apriltag_pose_matrix[2,3] *= 0.01
        #self.apriltag_pose_matrix = np.eye(4)
        self._inv_apriltag_pose_matrix = np.linalg.inv(self.apriltag_pose_matrix)

        self._inv_apriltag_pose_matrix = np.eye(4)

        self._done = False

    async def on_depth_frame(self, device_id, depth_frame, point_cloud):
        if self._done:
            return
        #from src.python.vision.rgbd import _dump_point_cloud

        #print("point_cloud", point_cloud.shape)

        #A_hmd = R * A_camera
        #p_hmd = R * p_camera

        #p_hmd = A_hmd * p_apriltag
        #p_apriltag = A_hmd^-1 * p_hmd

        # Track how many points found in each cell
        count = np.zeros((self._grid_cells, self._grid_cells), dtype = int)

        # Convert all points to HMD space
        print(self.oakd_to_hmd_matrix)
        print("point=", point_cloud[0])
        point_cloud[:,1] *= -1.0    # invert y to convert from RHS -> LHS
        points = np.vstack([ point_cloud.T, np.ones(point_cloud.shape[0]) ])    # point_cloud [N,3] -> [4,N], with w = 1.0
        print("point=", points[:,0])
        hmd_points = np.dot(self.oakd_to_hmd_matrix, points).T                  # transform each point then convert result back to [N,4]
        print("point=",hmd_points[0])
        #_dump_point_cloud(hmd_points)

        # Filter out height value outliers using an arbitrary quantile
        yMax = np.quantile(hmd_points[:,1], q = 0.9)

        # Convert from OAKD -> AprilTag local space to get the x and z
        # coordinates (we assume AprilTag is oriented perpendicular to floor)
        # TODO: may need to send over adjusted pose from HMD to enforce this constraint
        #aligned_points = np.dot(self._inv_apriltag_pose_matrix, points).T
        aligned_points= hmd_points

        # Convert to floor space coordinates with height (x,height,z) and then
        # to heightmap cells (x_idx,height,z_idx) such that x,z=(0,0) is at map
        # center
        floor_coords = np.column_stack([ aligned_points[:,0], hmd_points[:,1], aligned_points[:,2] ])
        heightmap_points = floor_coords
        heightmap_points[:,0] = ((heightmap_points[:,0] + 0.5 * self._map_size) / self._cell_size)
        heightmap_points[:,2] = ((heightmap_points[:,2] + 0.5 * self._map_size) / self._cell_size)
        heightmap_points = heightmap_points[ (heightmap_points[:,0] >= 0) * (heightmap_points[:,2] >= 0) ]    # filter out coordinates outside map boundaries
        heightmap_points = heightmap_points[ (heightmap_points[:,0] < self._grid_cells) * (heightmap_points[:,2] < self._grid_cells) ]

        # Construct heightmap out of points
        for point in heightmap_points:
            x_idx, height, z_idx = int(point[0]), point[1], int(point[2])
            if height > 1:
            #if height > yMax:
                continue
            count[z_idx, x_idx] += 1
            self._heightmap[z_idx, x_idx] = np.maximum(self._heightmap[z_idx, x_idx], height)

        max_visualized_height = np.max(self._heightmap)
        colorized = cv2.applyColorMap(
            (255.0 * np.minimum(self._heightmap, max_visualized_height) / max_visualized_height).astype(np.uint8),
            cv2.COLORMAP_RAINBOW
        )
        colorized = cv2.resize(colorized, (self._grid_cells * 10, self._grid_cells * 10), interpolation = cv2.INTER_NEAREST)
        cv2.imshow("height", colorized)

        self._done = True
        if self.on_heightmap_generated is not None:
            print("Heightmap generated")
            self.on_heightmap_generated(heightmap = self._heightmap, sample_count = count, cell_size = self._cell_size)
        return

        # Print out the grid
        """
        with open("points.txt", "w") as fp:
            for z_idx in range(self._grid_cells):
                for x_idx in range(self._grid_cells):
                    x = x_idx * self._cell_size
                    z = z_idx * self._cell_size
                    y = height_map[z_idx, x_idx]
                    if y > 20e-2:
                        fp.write("%f %f %f 128 128 128\n" % (x * 100, y * 100, z * 100))

        self._done = True
        return
        """

        count_threshold = 1
        height_threshold = 0
        s = ""
        for z_idx in range(self._grid_cells):
            for x_idx in range(self._grid_cells):
                if count[z_idx, x_idx] >= count_threshold:
                    height = height_map[z_idx, x_idx]
                    if height >= height_threshold:
                        s += "*"
                    else:
                        s += "."
                else:
                    s += " "
            s += "\n"
        print("--")
        print(s)
