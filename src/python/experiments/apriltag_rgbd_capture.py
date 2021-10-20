#
# Run from base directory:
#
# python -m src.python.experiments.apriltags_rgbd_capture --output-dir=\projects\MRLaserTag
#
# This will produce two files in the specified output directory:
#
#   points.txt: RGBD point cloud with 1280*720 rows, each consisting of:
#
#     x y z r g b
#
#   poses.txt: Detected marker poses. The file format is:
#
#     number_of_markers
#     marker_size
#     m00 m01 m02 m03
#     m10 m11 m12 m13
#     m20 m21 m22 m23
#     m30 m31 m32 m33
#     marker_size
#     ...
#
# All distance units are in cm and the coordinate system is x-right, y-down,
# z-forward. The marker file contains N markers where each entry provides a
# size and pose matrix. To convert the pose matrix to a y-up coordinate system,
# like Unity's, negate all values in the second row (m10, m11, m12, m13).
#
# The RGBDPointCloudViewer Unity project will load these files and render the
# point cloud and all markers (as quads).
#

import argparse
import cv2
import depthai as dai
import numpy as np
import open3d as o3d
import os
import thirdparty.apriltag.python.apriltag as apriltag


def get_calibration_data(device):
    return {
        # TODO: hard-coded for now until Gen2 calibration APIs are exposed
        "rectification_rotation_right": np.array(
            [
                [0.999997, -0.002380, 0.001091],
                [0.002376, 0.999991, 0.003572],
                [-0.001100, -0.003569, 0.999993],
            ]
        ),
        "rectification_rotation_left": np.array(
            [
                [0.999962, 0.001013, -0.008683],
                [-0.001044, 0.999993, -0.003566],
                [0.008679, 0.003575, 0.999956],
            ]
        ),
        "intrinsic_left": np.array(
            [
                [860.511047, 0.000000, 643.752136],
                [0.000000, 860.908447, 405.638458],
                [0.000000, 0.000000, 1.000000],
            ]
        ),
        "intrinsic_right": np.array(
            [
                [856.328430, 0.000000, 637.074036],
                [0.000000, 856.887146, 411.665314],
                [0.000000, 0.000000, 1.000000],
            ]
        ),
        # Rotation of right camera w.r.t. left camera
        "rotation_right_wrt_left": np.array(
            [
                [0.999946, -0.003455, 0.009766],
                [0.003385, 0.999969, 0.007148],
                [-0.009791, -0.007114, 0.999927],
            ]
        ),
        # Position of the right camera center w.r.t. left camera center
        "translation_right_wrt_left": np.array([[-7.473700], [-0.007575], [0.064895]]),
        # TODO: is this is a linear array or a 2x7 matrix?
        "distortion_coefficients_left": np.array(
            [
                -5.084880,
                16.812593,
                0.002646,
                0.000153,
                -17.761698,
                -5.141418,
                17.031750,
                -17.972408,
                0.000000,
                0.000000,
                0.000000,
                0.000000,
                0.000000,
                0.000000,
            ]
        ),
        "distortion_coefficients_right": np.array(
            [
                -5.406587,
                17.819704,
                0.001201,
                -0.000315,
                -18.399618,
                -5.456296,
                18.012304,
                -18.583179,
                0.000000,
                0.000000,
                0.000000,
                0.000000,
                0.000000,
                0.000000,
            ]
        ),
        "intrinsic_rgb": np.array(
            [
                [1487.305298, 0.000000, 964.402771],
                [0.000000, 1485.423584, 542.023193],
                [0.000000, 0.000000, 1.000000],
            ]
        ),
        # Rotation of RGB camera w.r.t. right (I *think*)
        "rotation_rgb_wrt_right": np.array(
            [
                [0.999977, 0.000553, 0.006696],
                [-0.000558, 1.000000, 0.000683],
                [-0.006695, -0.000687, 0.999977],
            ]
        ),
        "distortion_coefficients_rgb": np.array(
            [
                0.574014,
                4.646818,
                0.001263,
                -0.000055,
                101.891617,
                0.226793,
                6.719795,
                97.158440,
                0.000000,
                0.000000,
                0.000000,
                0.000000,
                0.000000,
                0.000000,
            ]
        ),
        # Position of RGB camera center w.r.t. right (I *think*) camera center
        "translation_rgb_wrt_right": np.array([[-3.768826], [0.056432], [0.135862]]),
    }


def compute_depth_to_rgb_conversion_matrices(device):
    calibration = get_calibration_data(device)

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


def create_rgb_and_depth_pipeline():
    pipeline = dai.Pipeline()

    cam_left = pipeline.createMonoCamera()
    cam_right = pipeline.createMonoCamera()
    cam_rgb = pipeline.createColorCamera()
    stereo = pipeline.createStereoDepth()
    xout_depth = pipeline.createXLinkOut()
    xout_video = pipeline.createXLinkOut()

    cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    for cam in [cam_left, cam_right]:
        cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        # cam.setFps(20.0)

    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(True)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

    xout_video.setStreamName("rgb_video")
    cam_rgb.video.link(xout_video.input)

    stereo.setOutputDepth(True)
    stereo.setOutputRectified(False)  # TODO: ask Luxonis about why this is not set
    stereo.setConfidenceThreshold(200)
    stereo.setRectifyEdgeFillColor(0)  # black, to better see the cutout
    stereo.setMedianFilter(
        dai.StereoDepthProperties.MedianFilter.KERNEL_7x7
    )  # KERNEL_7x7 default
    stereo.setLeftRightCheck(False)  # TODO: ask Luxonis why this is not set
    stereo.setExtendedDisparity(False)  # TODO: ""
    stereo.setSubpixel(
        False
    )  # TODO: "" -- setting this may improve accuracy for longer distances?

    stereo.setInputResolution(1280, 720)

    xout_depth.setStreamName("depth")
    cam_left.out.link(stereo.left)
    cam_right.out.link(stereo.right)
    stereo.depth.link(xout_depth.input)

    return pipeline


def convert_to_cv2_frame(feed_name, frame_data):
    data, width, height = (
        frame_data.getData(),
        frame_data.getWidth(),
        frame_data.getHeight(),
    )
    if feed_name == "rgb_video":  # YUV NV12
        yuv = np.array(data).reshape((height * 3 // 2, width)).astype(np.uint8)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
    elif feed_name == "depth":
        # TODO: this contains FP16 with (lrcheck or extended or subpixel)
        return np.array(data).astype(np.uint8).view(np.uint16).reshape((height, width))
    else:
        # Refer to gen2-rgbd-projection project for how to handle different frame types
        raise RuntimeError("Unknown frame type: %s" % feed_name)


def homogeneous_pixel_coordinates(width, height):
    """
    Pixel in homogenous coordinate
    Returns:
      Pixel coordinate: [3, width * height]
    """
    x = np.linspace(0, width - 1, width).astype(np.int)
    y = np.linspace(0, height - 1, height).astype(np.int)
    [x, y] = np.meshgrid(x, y)
    return np.vstack((x.flatten(), y.flatten(), np.ones_like(x.flatten())))


def dump_rgbd(depth, rgb, intrinsic_matrix, april_tag_poses):
    global options
    point_cloud_filepath = os.path.join(options.output_dir, "points.txt")
    pose_filepath = os.path.join(options.output_dir, "poses.txt")
    points = homogeneous_pixel_coordinates(1280, 720)
    K_inverse = np.linalg.inv(intrinsic_matrix)
    points_3d = (
        np.dot(K_inverse, points) * depth.flatten() * 0.1
    )  # convert to cm, which is how camera was calibrated
    b = rgb[:, :, 0].flatten()
    g = rgb[:, :, 1].flatten()
    r = rgb[:, :, 2].flatten()
    data = np.vstack((points_3d, r, g, b)).T  # shape=(1280*720,6)
    np.savetxt(fname=point_cloud_filepath, X=data)  # x, y, z (in cm), r, g, b
    print("Dumped point cloud to %s" % point_cloud_filepath)
    with open(pose_filepath, "w") as fp:
        fp.write("%d\n" % len(april_tag_poses))
        for pose in april_tag_poses:
            fp.write("12.2\n")  # cm size
            for y in range(4):
                fp.write(
                    "%f %f %f %f\n" % (pose[y, 0], pose[y, 1], pose[y, 2], pose[y, 3])
                )
        print("Dumped %d AprilTag poses to %s" % (len(april_tag_poses), pose_filepath))


def solve_pose(detection, intrinsic_matrix, tag_size_cm):
    # World coordinate system: +x -> right, +y -> down, +z -> forward (away from camera)
    # Detector returns points in clock-wise winding from top-left. This appears to be
    # consistent and reliable.
    object_points = tag_size_cm * np.vstack(
        [
            [-0.5, -0.5, 0],  # top-left
            [0.5, -0.5, 0],  # top-right
            [0.5, 0.5, 0],  # bottom-right
            [-0.5, 0.5, 0],  # bottom-left
        ]
    )
    image_points = np.array(detection.corners)
    retval, rodrigues, translation, inliers = cv2.solvePnPRansac(
        objectPoints=object_points,
        imagePoints=image_points,
        cameraMatrix=intrinsic_matrix,
        distCoeffs=None,
        rvec=None,
        tvec=None,
        useExtrinsicGuess=False,
        iterationsCount=2000,
        reprojectionError=1.0,
        confidence=0.99,
        inliers=None,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    rotation_matrix, _ = cv2.Rodrigues(rodrigues)
    pose_matrix = np.vstack(
        [np.hstack([rotation_matrix, translation]), np.array([0, 0, 0, 1])]
    )
    return pose_matrix


def reproject(pose_matrix, intrinsic_matrix, tag_size_cm):
    object_points = tag_size_cm * np.vstack(
        [
            [-0.5, -0.5, 0],  # top-left
            [0.5, -0.5, 0],  # top-right
            [0.5, 0.5, 0],  # bottom-right
            [-0.5, 0.5, 0],  # bottom-left
        ]
    )
    world_points = []
    for i in range(4):
        object_point = object_points[i, :].reshape((3, 1))
        object_point = np.vstack([object_point, 1])  # turn into a 4x1 vector
        world_point = np.matmul(pose_matrix, object_point)[0:3, :]
        world_points.append(world_point)
    image_points = []
    for world_point in world_points:
        perspective = world_point / world_point[2, 0]  # divide by z
        image_point = np.matmul(intrinsic_matrix, perspective)[0:2, 0]
        image_points.append(image_point)
    return image_points


def capture_and_dump_rgbd_frames(
    device,
    K_depth_inverse,
    Xform_depth_to_rgb,
    K_rgb,
    apriltag_detector,
    apriltag_camera_params,
):
    global options
    pixel_coords = homogeneous_pixel_coordinates(width=1280, height=720)

    # Create a queue for each stream
    output_queues = []
    for feed_name in ["rgb_video", "depth"]:
        queue = device.getOutputQueue(feed_name, 8, blocking=False)
        output_queues.append(queue)

    depth_rgb = None

    # Window with mouse callback
    def depthMouseCB(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # checks mouse left button down condition
            print("distance = %f mm" % depth_rgb[y, x])

    cv2.namedWindow("depth")
    cv2.setMouseCallback("depth", depthMouseCB)

    # Grab frames
    while True:
        # Dequeue both a color and depth frame
        for queue in output_queues:
            feed_name = queue.getName()
            frame_data = queue.get()

            frame = convert_to_cv2_frame(feed_name=feed_name, frame_data=frame_data)

            if feed_name == "rgb_video":
                scale_width = 1280 / frame.shape[1]  # 1920x1080 -> 1280x720
                output_resolution = (
                    int(frame.shape[1] * scale_width),
                    int(frame.shape[0] * scale_width),
                )
                color_frame = cv2.resize(
                    src=frame, dsize=output_resolution, interpolation=cv2.INTER_CUBIC
                )  # can change interpolation if needed to reduce computations
            elif feed_name == "depth":
                depth_frame = frame.copy()

        # Require both color and depth
        if depth_frame is None or color_frame is None:
            continue

        temp = depth_frame.copy()  # depth in right frame
        cam_coords = (
            np.dot(K_depth_inverse, pixel_coords) * temp.flatten() * 0.1    # mm (depth units) -> cm (units camera was calibrated at)
        )  # [x, y, z]
        del temp

        # cam_coords[:, cam_coords[2] > 1500] = float('inf')
        # o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cam_coords.transpose())
        pcd.remove_non_finite_points()
        pcd.transform(Xform_depth_to_rgb)

        rgb_frame_ref_cloud = np.asarray(pcd.points).transpose()
        rgb_frame_ref_cloud_normalized = rgb_frame_ref_cloud / rgb_frame_ref_cloud[2, :]
        rgb_image_pts = np.matmul(K_rgb, rgb_frame_ref_cloud_normalized)
        rgb_image_pts = rgb_image_pts.astype(np.int16)
        u_v_z = np.vstack((rgb_image_pts, rgb_frame_ref_cloud[2, :]))

        lft = np.logical_and(0 <= u_v_z[0], u_v_z[0] < 1280)
        rgt = np.logical_and(0 <= u_v_z[1], u_v_z[1] < 720)
        idx_bool = np.logical_and(lft, rgt)
        u_v_z_sampled = u_v_z[:, np.where(idx_bool)]
        y_idx = u_v_z_sampled[1].astype(int)
        x_idx = u_v_z_sampled[0].astype(int)

        depth_rgb = np.full((720, 1280), 65535, dtype=np.uint16)
        depth_rgb[y_idx, x_idx] = u_v_z_sampled[3] * 10 # convert back to mm

        # end = time.time()
        # print('for loop Convertion time')
        # print(end - start)
        # cv2.imshow('rgb_depth', depth_rgb)

        depth_rgb[depth_rgb == 0] = 65535

        # Colorize depth map
        max_visualized_depth = 10 * 1000.0  # 10m
        im_color = cv2.applyColorMap(
            (
                255.0
                * np.minimum(depth_rgb, max_visualized_depth)
                / max_visualized_depth
            ).astype(np.uint8),
            cv2.COLORMAP_RAINBOW,
        )
        cv2.imshow("depth", im_color)

        # Combine colorized depth map and RGB image
        added_image = cv2.addWeighted(color_frame, 0.6, im_color, 0.3, 0)

        # Detect AprilTags and draw their edges using different colors to observe whether point ordering is consistent
        gray_frame = cv2.cvtColor(color_frame, cv2.COLOR_RGB2GRAY)
        detections = apriltag_detector.detect(img=gray_frame)
        poses = []
        for detection in detections:
            for i in range(4):
                start, end = detection.corners[i], detection.corners[(i + 1) % 4]
                line_color = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)][
                    i
                ]  # red, green, blue, yellow (OpenCV uses BGR)
                cv2.line(
                    img=added_image,
                    pt1=(int(start[0]), int(start[1])),
                    pt2=(int(end[0]), int(end[1])),
                    color=line_color,
                    thickness=3,
                )

            # Get pose
            if options.cv_pose:
                M = solve_pose(
                    detection=detection, intrinsic_matrix=K_rgb, tag_size_cm=12.2
                )
            else:
                M, init_error, final_error = apriltag_detector.detection_pose(
                    detection=detection,
                    camera_params=apriltag_camera_params,
                    tag_size=12.2,
                )
            pose = np.array(M)
            # print(pose)
            poses.append(pose)

            # Reproject pose back into image space and render in cyan
            image_points = reproject(
                pose_matrix=pose, intrinsic_matrix=K_rgb, tag_size_cm=12.2
            )
            for i in range(4):
                start, end = image_points[i], image_points[(i + 1) % 4]
                cv2.line(
                    img=added_image,
                    pt1=(int(start[0]), int(start[1])),
                    pt2=(int(end[0]), int(end[1])),
                    color=(255, 255, 0),
                    thickness=2,
                )

        # Show combined image
        cv2.imshow("RGBD overlay ", added_image)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break
        elif key == ord("c"):
            dump_rgbd(
                depth=depth_rgb,
                rgb=color_frame,
                intrinsic_matrix=K_rgb,
                april_tag_poses=poses,
            )


if __name__ == "__main__":
    global options
    parser = argparse.ArgumentParser("apriltag_rgbd_capture")
    parser.add_argument(
        "--output-dir",
        metavar="path",
        type=str,
        action="store",
        required=True,
        help="Directory to write RGBD point cloud (points.txt) and AprilTag poses (poses.txt) to",
    )
    parser.add_argument(
        "--cv-pose",
        action="store_true",
        help="Use OpenCV PnP solver rather than AprilTag library for pose detection",
    )
    options = parser.parse_args()

    # AprilTag detector (quad_contours detection must be disabled or it will crash)
    apriltag_detector = apriltag.Detector(
        options=apriltag.DetectorOptions(
            quad_contours=False, refine_pose=True, quad_decimate=0
        ),
        searchpath=["./bin/"],
    )

    # TODO: make dir and get last captured frame number
    pipeline = create_rgb_and_depth_pipeline()
    with dai.Device(pipeline) as device:
        (
            K_depth_inverse,
            Xform_depth_to_rgb,
            K_rgb,
        ) = compute_depth_to_rgb_conversion_matrices(device)
        apriltag_camera_params = [
            K_rgb[0, 0],
            K_rgb[1, 1],
            K_rgb[0, 2],
            K_rgb[1, 2],
        ]  # fx, fy, cx, cy
        device.startPipeline()
        capture_and_dump_rgbd_frames(
            device,
            K_depth_inverse=K_depth_inverse,
            Xform_depth_to_rgb=Xform_depth_to_rgb,
            K_rgb=K_rgb,
            apriltag_detector=apriltag_detector,
            apriltag_camera_params=apriltag_camera_params,
        )
