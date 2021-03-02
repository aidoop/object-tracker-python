from aidoop.mask_rcnn import model as modellib, utils
from aidoop.mask_rcnn.config import Config
import os
import sys
import json
import datetime
import numpy as np
import skimage.draw
import time
import cv2
import cv2.aruco as aruco

############################################################
#  Inference Configurations
############################################################


class InferenceConfig(Config):
    """Configuration for training on the toy  dataset.
    Derives from the base Config class and overrides some values.
    """

    # Give the configuration a recognizable name
    NAME = "object-train"

    # We use a GPU with 12GB memory, which can fit two images.
    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 1

    # Number of classes (including background)
    NUM_CLASSES = 1 + 1  # Background + balloon

    # Skip detections with < 70% confidence
    DETECTION_MIN_CONFIDENCE = 0.7

    # Backbone network architecture
    # Supported values are: resnet50, resnet101.
    # You can also provide a callable that should have the signature
    # of model.resnet_graph. If you do so, you need to supply a callable
    # to COMPUTE_BACKBONE_SHAPE as well
    BACKBONE = "resnet101"  # "resnet101" or "resnet50"


class MaskRcnnDetect:

    PRINT_DETECTION_TIME = False

    def __init__(self, weight_file_path, log_dir):
        assert (weight_file_path is not None) and (
            log_dir is not None
        ), "weight_file_path and log_dir should not be empty"
        self.weight_file_path = weight_file_path
        self.log_dir = log_dir

        # initialize mask rcnn
        self.inference_config = InferenceConfig()
        self.inference_config.display()

        # create a model
        self.model = modellib.MaskRCNN(
            mode="inference", config=self.inference_config, model_dir=self.log_dir
        )
        assert self.model is not None

        self.model.load_weights(self.weight_file_path, by_name=True)

        # mask list
        self.mask_list = list()

        # scores
        self.scores = list()

    def detect_object_by_path(self, image_path):
        assert image_path is not None

        # Read image
        image = skimage.io.imread(image_path)

        # Detect objects
        sttime = time.time()
        result = self.model.detect([image], verbose=0)[0]
        edtime = time.time()

        if MaskRcnnDetect.PRINT_DETECTION_TIME:
            print("-----------------------------------------------------")
            print("MRCNN Elapsed Time: {} sec".format(edtime - sttime))
            print("Object Count: ", result["masks"].shape[-1])

        self.mask_list = self.get_mask_list(result["masks"])
        self.scores = result["scores"]

        return self.mask_list

        # # Color splash
        # splash = get_mask_by_index(image, result['masks'], 0)

        # # Save output
        # file_name = "splash_{:%Y%m%dT%H%M%S}.png".format(
        #     datetime.datetime.now())
        # skimage.io.imsave(file_name, splash)
        # print("Saved to ", file_name)

        # return splash

    def detect_object_by_data(self, image_data):
        assert image_data is not None, "image_data should not be empty"

        # Detect objects
        sttime = time.time()
        result = self.model.detect([image_data], verbose=0)[0]
        edtime = time.time()

        if MaskRcnnDetect.PRINT_DETECTION_TIME:
            print("-----------------------------------------------------")
            print("MRCNN Elapsed Time: {} sec".format(edtime - sttime))
            print("Object Count: ", result["masks"].shape[-1])

        self.mask_list = self.get_mask_list(result["masks"])
        self.scores = result["scores"]

        return self.mask_list

    def get_mask_list(self, masks):

        # mask_cnt = masks.shape[-1]
        # for idx in range(mask_cnt):
        #     mask = masks[:, :, 0]
        #     mask.reshape(mask.shape[0], mask.shape[1], 1)
        #     mask_gray = np.where(mask, 255, 0).astype(np.uint8)

        mask_list = list()

        # get mask count
        mask_cnt = masks.shape[-1]

        # make the list of mask
        for mask_idx in range(mask_cnt):
            mask = masks[:, :, mask_idx]
            mask_list.append(mask)

        return mask_list

    def get_center_points(self, mask_list):
        center_point_list = list()

        if len(mask_list) <= 0:
            return center_point_list

        if len(mask_list) > 0:
            accumulated_mask = mask_list[0]
            for mask in mask_list:
                accumulated_mask = np.logical_or(mask, accumulated_mask)

            mask_image = np.where(accumulated_mask, 255, 0).astype(np.uint8)

            # get the center point of mask regions
            contours, hierarchy = cv2.findContours(
                mask_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                # calculate moments for each contour
                M = cv2.moments(contour)

                if M["m00"] == 0.0:
                    continue

                # calculate x,y coordinate of center
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                center_point_list.append((cX, cY))

        return center_point_list

    def get_mask_image(self, mask_list, width, height):
        # center_point_list = list()
        if len(mask_list) > 0:
            accumulated_mask = mask_list[0]
            for mask in mask_list:
                accumulated_mask = np.logical_or(mask, accumulated_mask)

            mask_image = np.where(accumulated_mask, 255, 0).astype(np.uint8)

            # # get the center point of mask regions
            # contours, hierarchy = cv2.findContours(
            #     mask_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            # for contour in contours:
            #     # calculate moments for each contour
            #     M = cv2.moments(contour)

            #     if M["m00"] == 0.0:
            #         continue

            #     # calculate x,y coordinate of center
            #     cX = int(M["m10"] / M["m00"])
            #     cY = int(M["m01"] / M["m00"])

            #     center_point_list.append((cX, cY))

            # skip to draw the ceter point
            # cv2.circle(mask_image, (cX, cY), 5, (0, 0, 0), -1)
        else:
            mask_image = np.zeros((height, width))

        return mask_image.astype(np.uint8)

    def get_splash_image_by_index(self, image, masks, mask_index):

        # Copy color pixels from the original color image where mask is set
        if masks.shape[-1] > 0:
            # We're treating all instances as one, so collapse the mask into one layer
            # mask = (np.sum(mask, -1, keepdims=True) >= 1)
            mask = masks[:, :, mask_index]
            mask.reshape(mask.shape[0], mask.shape[1], 1)
            splash = np.where(mask, 255, 0).astype(np.uint8)
        else:
            splash = gray.astype(np.uint8)
        return splash

    def get_scores(self):
        return self.scores


############################################################
#  Inference Example
############################################################
if __name__ == "__main__":
    import argparse

    # Root directory of the project
    ROOT_DIR = os.path.abspath("./")

    # Import Mask RCNN
    sys.path.append(ROOT_DIR)  # To find local version of the library

    # Path to trained weights file
    COCO_WEIGHTS_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201006T1521/mask_rcnn_object-train_0047.h5"

    # Directory to save logs and model checkpoints, if not provided
    # through the command line argument --logs
    DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")

    EXAMPLE_IMAGE_PATH = os.path.join(
        ROOT_DIR, "dataset", "balloon", "val", "14898532020_ba6199dd22_k.jpg"
    )

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="infer Mask R-CNN to detect an object."
    )
    parser.add_argument(
        "--weights",
        required=True,
        metavar="/path/to/weights.h5",
        help="Path to weights .h5 file or 'coco'",
    )
    parser.add_argument(
        "--image",
        required=True,
        metavar="path or URL to image",
        help="Image to apply the color splash effect on",
    )
    args = parser.parse_args()

    # print("Weights: ", args.weights)
    # print("Dataset: ", args.dataset)
    # print("Logs: ", args.logs)

    mrdetect = MaskRcnnDetect(COCO_WEIGHTS_PATH, DEFAULT_LOGS_DIR)

    mrdetect.detect_object_by_path(EXAMPLE_IMAGE_PATH)
