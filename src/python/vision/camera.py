import asyncio
import cv2
import depthai as dai
import numpy as np
import os
from pathlib import Path
import time
import weakref

from src.python.vision.rgbd import RGBDFrameConverter


def _get_calibration_data(device):
    calibration_handler = device.readCalibration()

    #
    # rectification_rotation_right vs. _left appear to be opposite of the hard-
    # coded values I had extracted using the Gen1 API. Likewise,
    # extrinsic_rgb_wrt_right looks opposite as well.
    #
    # For now, I am using the rectification rotations supplied by the API but
    # have adjusted the extrinsic to be the opposite of what is implied by the
    # name. This appears to give a reasonable result.
    #
    # Registration seems relatively insensitive to flipping the extrinsic
    # parameters (although this needs more investigation) but appears more
    # sensitive to the rectification rotations being swapped. I'm not sure
    # where the error is. Did I misread the Gen1 API calibration results? Is my
    # RGBD math wrong? Or, did Luxonis make a mistake in the Gen1 or Gen2 API?
    #
    calibration = {}
    calibration["intrinsic_rgb"] = np.array(calibration_handler.getCameraIntrinsics(cameraId = dai.CameraBoardSocket.RGB))
    calibration["intrinsic_left"] = np.array(calibration_handler.getCameraIntrinsics(cameraId = dai.CameraBoardSocket.LEFT))
    calibration["intrinsic_right"] = np.array(calibration_handler.getCameraIntrinsics(cameraId = dai.CameraBoardSocket.RIGHT))
    calibration["rectification_rotation_right"] = np.array(calibration_handler.getStereoRightRectificationRotation())   # why are the left/right results swapped from what I had before?
    calibration["rectification_rotation_left"] = np.array(calibration_handler.getStereoLeftRectificationRotation())

    extrinsic_right_wrt_left = np.array(calibration_handler.getCameraExtrinsics(srcCamera = dai.CameraBoardSocket.LEFT, dstCamera = dai.CameraBoardSocket.RIGHT))
    calibration["rotation_right_wrt_left"] = extrinsic_right_wrt_left[0:3,0:3]
    calibration["translation_right_wrt_left"] = extrinsic_right_wrt_left[0:3,3].reshape((1, 3)).T

    extrinsic_rgb_wrt_right = np.array(calibration_handler.getCameraExtrinsics(srcCamera = dai.CameraBoardSocket.RGB, dstCamera = dai.CameraBoardSocket.RIGHT)) # other way around?
    calibration["rotation_rgb_wrt_right"] = extrinsic_rgb_wrt_right[0:3,0:3]
    calibration["translation_rgb_wrt_right"] = extrinsic_rgb_wrt_right[0:3,3].reshape((1, 3)).T

    calibration["horizontal_field_of_view_rgb"] = calibration_handler.getFov(cameraId = dai.CameraBoardSocket.RGB)

    return calibration


def _get_hard_coded_calibration_data():
    return {
        # Hard-coded for device_id=14442C10512EC0D200, obtained via Gen1 API
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


def convert_rgb_camera_frame_to_cv2(frame_data):
    data, width, height = (
        frame_data.getData(),
        frame_data.getWidth(),
        frame_data.getHeight(),
    )
    yuv = np.array(data).reshape((height * 3 // 2, width)).astype(np.uint8)
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)


def convert_depth_frame_to_cv2(frame_data):
    data, width, height = (
        frame_data.getData(),
        frame_data.getWidth(),
        frame_data.getHeight(),
    )
    # TODO: this contains FP16 with (lrcheck or extended or subpixel)
    return np.array(data).astype(np.uint8).view(np.uint16).reshape((height, width))


def _screenshot(color_frame):
    def try_parse_int(value):
        try:
            return int(value)
        except ValueError:
            return None

    # Iterate existing images to find next number
    basenames = [ os.path.splitext(path)[0] for path in Path(".").glob("capture_*.jpg") ]
    numbers = [ try_parse_int(basename.split("_")[1]) for basename in basenames ]
    numbers = [ number for number in numbers if (number is not None and number >= 0) ]
    next_number = 0 if len(numbers) == 0 else (max(numbers) + 1)

    # Write to disk
    filename = "capture_%d.jpg" % next_number
    cv2.imwrite(filename, color_frame)
    print("Saved color frame: %s" % filename)


class CameraTask:
    """
    Responsible for acquiring and distributing RGB and depth frames for a
    particular OAKD device.
    """
    def __init__(self, device_id, main_camera_registration_matrix = np.eye(4), window_size = (1920, 1080), show_fps = False):
        """
        Create a task corresponding to a particular OAKD camera.

        Parameters
        ----------
        device_id : str
            DepthAI identifier string for OAKD camera.
        main_camera_registration_matrix : np.ndarray
            A 4x4 transformation matrix from this camera's space to the
            main camera's space. Note that the translation units are in cm
            and that therefore, the matrix can be used to transform
            AprilTag poses (which are also in cm) directly.
        window_size : (int, int)
            Size of the window to render RGB stream to.
        show_fps : bool
            Whether or not to render the frame rate counter in the RGB
            output.
        """

        self.device_id = device_id
        self.main_camera_registration_matrix = main_camera_registration_matrix
        self._window_size = window_size
        self._show_fps = show_fps

        # Calibration
        found, device_info = dai.Device.getDeviceByMxId(device_id)
        if not found:
            raise RuntimeError("Device %s not found" % device_id)
        with dai.Device(dai.Pipeline(), device_info) as device:
            self.calibration = _get_calibration_data(device = device)

        # RGBD frame converter
        self._rgbd_converter = RGBDFrameConverter(calibration = self.calibration, width = 1280, height = 720)

        # Frame subscribers
        self._rgb_subscribers = []
        self._depth_subscribers = []

        # Text labels on the main window for tasks to output messages. Each is
        # a weak reference to a CameraTask.TextLabel.
        self._text_labels = []

        # Video writer: writes at full 1080p resolution
        self._video = None

    def start_video_recording(self, video_path):
        if self._video is None:
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            self._video = cv2.VideoWriter(video_path, fourcc, 30.0, (1920,1080))
            print("Started recording video to %s" % video_path)

    def add_rgb_subscriber(self, subscriber):
        """
        Adds a subscriber for RGB frame data.

        Parameters
        ----------
        subscriber : object
            A subscriber with an async on_rgb_frame(rgb_data, color_frame)
            method. The object is retained as a weak reference. The caller
            must keep it alive.
        """
        existing = [ existing_subscriber for existing_subscriber in self._rgb_subscribers if existing_subscriber() == subscriber ]
        if len(existing) > 0:
            return
        self._rgb_subscribers.append(weakref.ref(subscriber))

    def remove_rgb_subscriber(self, subscriber):
        """
        Removes an RGB frame subscriber.

        Parameters
        ----------
        subscriber : object
            Strong reference to subscriber object to remove.
        """
        self._rgb_subscribers = [ existing_subscriber for existing_subscriber in self._rgb_subscribers if existing_subscriber() != subscriber ]

    def add_depth_subscriber(self, subscriber):
        """
        Adds a subscriber for depth frame (in RGB camera space) data.

        Parameters
        ----------
        subscriber : object
            A subscriber with an async on_depth_frame(depth_frame,
            point_cloud) method. The object is retained as a weak
            reference. The caller must keep it alive.
        """
        existing = [ existing_subscriber for existing_subscriber in self._depth_subscribers if existing_subscriber() == subscriber ]
        if len(existing) > 0:
            return
        self._depth_subscribers.append(weakref.ref(subscriber))

    def remove_depth_subscriber(self, subscriber):
        """
        Removes a depth frame subscriber.

        Parameters
        ----------
        subscriber : object
            Strong reference to subscriber object to remove.
        """
        self._depth_subscribers = [ existing_subscriber for existing_subscriber in self._depth_subscribers if existing_subscriber() != subscriber ]

    async def run(self, loop):
        pipeline = self._create_rgbd_pipeline()
        found, device_info = dai.Device.getDeviceByMxId(self.device_id)
        if not found:
            raise RuntimeError("Device %s not found" % self.device_id)
        with dai.Device(pipeline, device_info) as device:
            _get_calibration_data(device)
            device.startPipeline()
            rgb_queue = device.getOutputQueue("rgb_video", 16, blocking = False)
            depth_queue = device.getOutputQueue("depth", 16, blocking = False)
            await self._run_camera(rgb_queue = rgb_queue, depth_queue = depth_queue)
        loop.stop()
        print("Finished")

    class TextLabel:
        def __init__(self):
            self.text = ""

    def createTextLabel(self):
        """
        Create a text label, a line on the main camera view on which text can
        be displayed by any task.

        Returns
        -------
        CameraTask.TextLabel
            A TextLabel object.
        """
        text_label = CameraTask.TextLabel()
        text_label_ref = weakref.ref(text_label)
        self._text_labels.append(text_label_ref)
        return text_label

    def _create_rgb_pipeline(self):
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.createColorCamera()
        xout_video = pipeline.createXLinkOut()
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(True)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        xout_video.setStreamName("rgb_video")
        cam_rgb.video.link(xout_video.input)
        return pipeline

    def _create_rgbd_pipeline(self):
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

    async def _run_camera(self, rgb_queue, depth_queue):
        lastFrameTime = time.perf_counter()
        while True:
            rgb_data = None
            depth_data = None
            color_frame = None  # full RGB color frame

            # Purge dead subscribers
            self._rgb_frame_subscribers = [ subscriber for subscriber in self._rgb_subscribers if subscriber() is not None ]
            self._depth_frame_subscribers = [ subscriber for subscriber in self._depth_subscribers if subscriber() is not None ]

            # Do we need to perform any work that requires a frame?
            rgb_frame_required = len(self._rgb_subscribers) > 0 or self._video is not None
            depth_frame_required = len(self._depth_subscribers) > 0

            if rgb_frame_required:
                rgb_data = rgb_queue.get()
                color_frame = convert_rgb_camera_frame_to_cv2(frame_data = rgb_data)

                # Send to subscribers
                for subscriber in self._rgb_subscribers:
                    subscriber = subscriber()
                    if subscriber is not None:
                        await subscriber.on_rgb_frame(device_id = self.device_id, rgb_data = rgb_data, color_frame = color_frame)

                # Clean up
                del rgb_data

            if depth_frame_required:
                depth_data = depth_queue.get()
                depth_frame = convert_depth_frame_to_cv2(frame_data = depth_data)

                # Convert to RGB space and return depth for each pixel as well as a point cloud
                depth_rgb_frame, point_cloud = self._rgbd_converter.convert_to_rgb_coordinate_system(depth_frame = depth_frame)

                # Send to subscribers
                for subscriber in self._depth_subscribers:
                    subscriber = subscriber()
                    if subscriber is not None:
                        await subscriber.on_depth_frame(device_id = self.device_id, depth_frame = depth_rgb_frame, point_cloud = point_cloud)

                # Colorize depth map
                max_visualized_depth = 10 * 1000.0  # 10m
                colorized_frame = cv2.applyColorMap(
                    (255.0 * np.minimum(depth_rgb_frame, max_visualized_depth) / max_visualized_depth).astype(np.uint8),
                    cv2.COLORMAP_RAINBOW,
                )
                cv2.imshow("depth", colorized_frame)

                # Clean up
                del depth_rgb_frame
                del point_cloud

            # Display frame
            if color_frame is not None:
                # Frame timing
                now = time.perf_counter()
                frameRateHz = 1.0 / (now - lastFrameTime)
                lastFrameTime = now
                if self._show_fps:
                    cv2.putText(color_frame, "%1.1f" % frameRateHz, org = (50, 50), fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 1, color = (0, 255, 255), thickness = 2, lineType = cv2.LINE_AA)

                # Text labels
                self._update_text_labels(color_frame = color_frame)

                # Write to video file
                if self._video is not None:
                    self._video.write(color_frame)

                # Display
                if self._window_size != (1920, 1080):
                    resized_color_frame = cv2.resize(src = color_frame, dsize = self._window_size)
                    cv2.imshow("RGB - %s" % self.device_id, resized_color_frame)
                    del resized_color_frame
                else:
                    cv2.imshow("RGB - %s" % self.device_id, color_frame)

                # Process input
                key = cv2.waitKey(1)
                if key == ord("q"):
                    return
                elif key == ord("p"):
                    _screenshot(color_frame = color_frame)

                del color_frame

            # Yield
            await asyncio.sleep(0)

    def _update_text_labels(self, color_frame):
        self._text_labels = [ text_label for text_label in self._text_labels if text_label() is not None ]
        text_labels = [ text_label() for text_label in self._text_labels ]

        # Precompute total height so we can position labels at bottom of window
        spacing = 10
        total_height = 0
        line_heights = []
        for text_label in text_labels:
            (width, height), baseline = cv2.getTextSize(text_label.text, fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 1, thickness = 2)
            line_height = height + baseline + spacing
            line_heights.append(line_height)
            total_height += line_height

        # Write text lines
        y = color_frame.shape[0] - total_height
        for i in range(len(text_labels)):
            text_label = text_labels[i]
            cv2.putText(color_frame, text_label.text, org = (50, y), fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 1, color = (255, 255, 255), thickness = 2, lineType = cv2.LINE_AA)
            y += line_heights[i]

        # Delete strong references
        del text_labels