##
## Mixed Reality Laser Tag
## Copyright 2021 Bart Trzynadlowski
##
## classifier.py
##
## Object classifier process. Run in an environment with Python 3.6.13 and
## packages from thirdparty/maskrcnn/requirements.txt.
##

import argparse
import asyncio
import json
import numpy as np
import os
import sys

import pywintypes
import win32event
from mmapfile import mmapfile


###############################################################################
# Point Cloud Labeling
###############################################################################

def label_point_cloud(point_cloud, masks, labels):
    labels_and_points = []
    for i in range(masks.shape[2]):
        mask = masks[:,:,i].flatten()                   # linear array of size 1280*720, just like the second point cloud axis
        points = point_cloud[:, np.where(mask != 0)[0]] # corresponding points
        if points.size > 0:
            label = labels[i]
            labels_and_points.append((label, points))
    return labels_and_points


###############################################################################
# MaskRCNN
###############################################################################

# MaskRCNN imports
sys.path.append("thirdparty/maskrcnn")  # MaskRCNN path root
sys.path.append("thirdparty/maskrcnn/samples/coco")
from thirdparty.maskrcnn.mrcnn import utils
import thirdparty.maskrcnn.mrcnn.model as modellib
from thirdparty.maskrcnn.mrcnn import visualize
import coco

class MaskRCNN:
    def __init__(self):
        # Load weights
        coco_model_path = os.path.join("assets", "mask_rcnn_coco.h5")
        if not os.path.exists(coco_model_path):
            utils.download_trained_weights(coco_model_path)

        class InferenceConfig(coco.CocoConfig):
            GPU_COUNT = 1
            IMAGES_PER_GPU = 1
        config = InferenceConfig()

        self._model = modellib.MaskRCNN(mode = "inference", model_dir = "assets", config = config)
        self._model.load_weights(coco_model_path, by_name = True)

        self.class_names = [
            'BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
            'bus', 'train', 'truck', 'boat', 'traffic light',
            'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
            'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
            'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
            'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
            'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
            'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
            'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
            'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
            'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
            'teddy bear', 'hair drier', 'toothbrush'
        ]

    def infer(self, image_data):
        results = self._model.detect([image_data], verbose = 1)
        return results[0]


###############################################################################
# Server Task
###############################################################################

class ClassifierServer:
    def __init__(self, visualize):
        self._maskrcnn = MaskRCNN()
        self._visualize = options.visualize
        self._create_shared_memory()

    async def run_server(self):
        while True:
            print("Running...")

            # Wait for client to hand us a job
            timeout_ms = 32
            while win32event.WaitForSingleObject(self._server_event_handle, timeout_ms) != win32event.WAIT_OBJECT_0:
                await asyncio.sleep(0)  # let other tasks run while we wait
            print("Got a job!")

            # Extract data
            self._memory_handle.seek(0, 0)
            client_id = int.from_bytes(bytes = self._memory_handle.read(4), byteorder = "little")
            completed = int.from_bytes(bytes = self._memory_handle.read(4), byteorder = "little")
            num_rgb_bytes = int.from_bytes(bytes = self._memory_handle.read(4), byteorder = "little")
            rgb_bytes = self._memory_handle.read(num_rgb_bytes)
            num_point_cloud_bytes = int.from_bytes(bytes = self._memory_handle.read(4), byteorder = "little")
            point_cloud_bytes = self._memory_handle.read(num_point_cloud_bytes)
            print("client_id=%d, completed=%d, num_rgb_bytes=%d, num_point_cloud_bytes=%d" % (client_id, completed, num_rgb_bytes, num_point_cloud_bytes))

            # Classify
            image_data = np.frombuffer(buffer = rgb_bytes, dtype = np.uint8).reshape((720, 1280, 3))
            results = self._maskrcnn.infer(image_data = image_data)
            print("masks=", results["masks"].shape)

            # Label the point cloud
            point_cloud = np.frombuffer(buffer = point_cloud_bytes, dtype = float).reshape((3, 720 * 1280))
            labels_and_points = label_point_cloud(point_cloud = point_cloud, masks = results["masks"], labels = results["class_ids"])

            # Visualize
            if self._visualize:
                visualize.display_instances(image_data, results['rois'], results['masks'], results['class_ids'], self._maskrcnn.class_names, results['scores'])

            # Serialize the response: JSON list of instance labels followed by each point cloud in the same order
            labels = [ self._maskrcnn.class_names[label_idx] for label_idx, _ in labels_and_points ]    # array of labels, in order
            labels_bytes = json.JSONEncoder().encode(labels).encode("utf-8")
            self._memory_handle.seek(4, 0)                                      # leave client_id intact
            self._memory_handle.write(int(1).to_bytes(4, byteorder = "little")) # mark job as completed
            self._memory_handle.write(len(labels_bytes).to_bytes(4, byteorder = "little"))
            self._memory_handle.write(labels_bytes)
            print("--")
            for label_idx, points in labels_and_points:
                points_bytes = points.tobytes(order = "C")
                self._memory_handle.write(len(points_bytes).to_bytes(4, byteorder = "little"))
                self._memory_handle.write(points_bytes)

            # Signal to client that the response is ready
            win32event.SetEvent(self._client_event_handle)

    def _create_shared_memory(self):
        server_event_name = "Local\\MRLaserTag_Classifier_Producer" # server waits on client to pulse this to obtain job
        client_event_name = "Local\\MRLaserTag_Classifier_Consumer" # client waits on server to pulse this to obtain result of last job
        memory_name = "Local\\MRLaserTag_Classifier_Memory"
        memory_size = 4*4 + 1280*720*3*1 + 1280*720*3*8

        try:
            self._server_event_handle = win32event.CreateEvent(None, False, False, server_event_name)
        except pywintypes.error as error:
            print("Failed to create server event: %s" % error.strerror)
            raise
        print("Created producer event")

        try:
            self._client_event_handle = win32event.CreateEvent(None, False, False, client_event_name)
        except pywintypes.error as error:
            print("Failed to create client event: %s" % error.strerror)
            raise
        print("Created consumer event")

        try:
            self._memory_handle = mmapfile(None, memory_name, memory_size, 0, 0)
        except pywintypes.error as error:
            print("Failed to create shared memory: %s" % error.strerror)
            raise
        print("Created shared memory")


###############################################################################
# Program Entry Point
###############################################################################

if __name__ == "__main__":
    parser = argparse.ArgumentParser("classifier")
    parser.add_argument("--visualize", action="store_true", help="Visualize MaskRCNN detection results")
    options = parser.parse_args()

    print("Classifier server started")

    loop = asyncio.get_event_loop()
    server = ClassifierServer(visualize = options.visualize)
    loop.create_task(server.run_server())
    loop.run_forever()