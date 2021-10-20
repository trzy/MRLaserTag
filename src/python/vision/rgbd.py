import numpy as np
import open3d as o3d

import src.python.vision.camera as camera


def compute_depth_to_rgb_conversion_matrices(calibration):
    R_rectify_right = calibration["rectification_rotation_right"]
    R_rgb_wrt_right = calibration["rotation_rgb_wrt_right"]
    K_right = calibration["intrinsic_right"]
    K_rgb = calibration["intrinsic_rgb"]
    T_rgb_wrt_right = calibration["translation_rgb_wrt_right"]

    # Mono cameras were calibrated at 1280x800 but being consumed at 1280x720
    # here. Need to adjust for sensor center position.
    K_right[1, 2] -= 0.5 * (800 - 720)

    # From homogeneous space (projected coordinates before the perspective
    # divide) back to right-camera world  space (3D, with the right camera at the
    # origin)
    K_right_inverse = np.linalg.inv(K_right)

    # RGB camera was calibrated at 1920x1080. We are computing depth at 1280x720
    # and to project into RGB camera space, we must scale the RGB intrinsics to
    # that resolution.
    scale_factor = 1280 / 1920
    scale_matrix = np.array([[scale_factor, 0, 0], [0, scale_factor, 0], [0, 0, 1]])
    K_rgb_scaled = np.matmul(scale_matrix, K_rgb)

    # Constructs a matrix that transforms a homogeneous coordinate from the
    # rectified right camera back to un-rectified right-camera world space. I
    # *think* depth is returned relative to the rectified right camera otherwise
    # this wouldn't make much sense. It is what Luxonis do in their code.
    R_unrectify_right = np.linalg.inv(R_rectify_right)
    K_depth_inverse = np.matmul(R_unrectify_right, K_right_inverse)

    # Construct a transform that converts a point in right-camera space to RGB-
    # camera space and, therefore, depth data back to RGB-camera space
    R_rgb_wrt_right_inverse = np.linalg.inv(R_rgb_wrt_right)
    T_rgb_wrt_right_inverse = -1.0 * T_rgb_wrt_right
    extrinsic_matrix = np.hstack((R_rgb_wrt_right_inverse, T_rgb_wrt_right_inverse))
    Xform_depth_to_rgb = np.vstack((extrinsic_matrix, np.array([0, 0, 0, 1])))

    return K_depth_inverse, Xform_depth_to_rgb, K_rgb_scaled


def homogeneous_pixel_coordinates(width, height):
    """
    Pixel in homogenous coordinate

    Returns
    -------
    np.ndarray
        Pixel coordinate: [3, width * height]
    """
    x = np.linspace(0, width - 1, width).astype(np.int)
    y = np.linspace(0, height - 1, height).astype(np.int)
    [x, y] = np.meshgrid(x, y)
    return np.vstack((x.flatten(), y.flatten(), np.ones_like(x.flatten())))


def _dump_point_cloud(point_cloud):
    point_cloud_filepath = "points.txt"
    with open(point_cloud_filepath, "w") as fp:
        for point in point_cloud:
            fp.write("%f %f %f 128 128 128\n" % (point[0] * 1e2, point[1] * 1e2, point[2] * 1e2))


def _dump_point_cloud_from_depth_frame(depth, intrinsic_matrix):
    point_cloud_filepath = "points.txt"
    points = homogeneous_pixel_coordinates(1280, 720)
    K_inverse = np.linalg.inv(intrinsic_matrix)
    points_3d = (
        np.dot(K_inverse, points) * depth.flatten() * 0.1
    )  # convert to cm, which is how camera was calibrated
    b = np.ones(1280*720).flatten() * 128
    g = np.ones(1280*720).flatten() * 128
    r = np.ones(1280*720).flatten() * 128
    data = np.vstack((points_3d, r, g, b)).T  # shape=(1280*720,6)
    np.savetxt(fname=point_cloud_filepath, X=data)  # x, y, z (in cm), r, g, b


class RGBDFrameConverter:
    def __init__(self, calibration, width, height):
        """
        Constructs a converter object that converts a depth frame from
        right camera to RGB camera space.

        Parameters
        ----------
        width : int
            Width of each RGB frame in pixels.
        height : int
            Height of each RGB frame in pixels.
        """
        assert width == 1280 and height == 720  # currently we are hard-coded for 1280x720
        self.width = width
        self.height = height
        self._pixel_coords = homogeneous_pixel_coordinates(width = width, height = height)
        self._K_depth_inverse, self._Xform_depth_to_rgb, self._K_rgb = compute_depth_to_rgb_conversion_matrices(calibration = calibration)

    def convert_to_rgb_coordinate_system(self, depth_frame):
        """
        Converts a depth frame from right camera space to RGB camera space.

        Parameters
        ----------
        depth_frame : np.ndarray
            The depth frame in right camera space as a tensor of shape
            (height,width). Each value is a depth measurement in mm.

        Returns
        -------
        np.ndarray, np.ndarray
            The depth frame mapped to RGB camera space as a tensor of shape
            (height, width), with each value is in mm, and a point cloud of
            shape (N, 3), with each 3-tuple being (x, y, z) in meters.
        """

        # Get depth points in right camera space, [x,y,z]
        temp = depth_frame.copy()
        cam_coords = (
            np.dot(self._K_depth_inverse, self._pixel_coords) * temp.flatten() * 0.1    # mm (depth units) -> cm (units camera was calibrated at)
        )  # [x, y, z]
        del temp

        # Convert to a point cloud in RGB camera space
        # cam_coords[:, cam_coords[2] > 1500] = float('inf')
        # o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cam_coords.transpose())
        pcd.remove_non_finite_points()
        pcd.transform(self._Xform_depth_to_rgb)

        # Convert to homogeneous coordinates in RGB camera space, [u,v,z]
        rgb_frame_ref_cloud = np.asarray(pcd.points).transpose()
        rgb_frame_ref_cloud_normalized = rgb_frame_ref_cloud / rgb_frame_ref_cloud[2, :]
        rgb_image_pts = np.matmul(self._K_rgb, rgb_frame_ref_cloud_normalized)
        rgb_image_pts = rgb_image_pts.astype(np.int16)
        u_v_z = np.vstack((rgb_image_pts, rgb_frame_ref_cloud[2, :]))

        # Filter out points that lie outside of the frame
        hor = np.logical_and(0 <= u_v_z[0], u_v_z[0] < self.width)
        ver = np.logical_and(0 <= u_v_z[1], u_v_z[1] < self.height)
        idx_bool = np.logical_and(hor, ver)
        u_v_z_sampled = u_v_z[:, np.where(idx_bool)]
        y_idx = u_v_z_sampled[1].astype(int)
        x_idx = u_v_z_sampled[0].astype(int)

        # Convert to a depth frame in RGB camera space
        depth_rgb = np.full((self.height, self.width), 65535, dtype = np.uint16)
        depth_rgb[y_idx, x_idx] = u_v_z_sampled[3] * 10 # convert back to mm

        # More garbage to filter out
        depth_rgb[depth_rgb == 0] = 65535

        # Point cloud as np array, cm -> m
        point_cloud = np.array(pcd.points) * 1e-2

        return depth_rgb, point_cloud