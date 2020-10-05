
from mrcnn import model as modellib, utils
from mrcnn.config import Config
import os
import sys
import json
import datetime
import numpy as np
import skimage.draw

import time

# Root directory of the project
ROOT_DIR = os.path.abspath("../")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library

# Path to trained weights file
COCO_WEIGHTS_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")

# Directory to save logs and model checkpoints, if not provided
# through the command line argument --logs
DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")

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

    # Number of training steps per epoch
    STEPS_PER_EPOCH = 100

    # Skip detections with < 90% confidence
    DETECTION_MIN_CONFIDENCE = 0.9


def get_mask(image, masks, mask_index):
    """
    mask: instance segmentation mask [height, width, instance count]
    Returns mask image.
    """

    # mask_cnt = masks.shape[-1]
    # for idx in range(mask_cnt):
    #     mask = masks[:, :, 0]
    #     mask.reshape(mask.shape[0], mask.shape[1], 1)
    #     mask_gray = np.where(mask, 255, 0).astype(np.uint8)

    # Make a grayscale copy of the image. The grayscale copy still
    # has 3 RGB channels, though.
    gray = skimage.color.gray2rgb(skimage.color.rgb2gray(image)) * 255
    # Copy color pixels from the original color image where mask is set
    if masks.shape[-1] > 0:
        # We're treating all instances as one, so collapse the mask into one layer
        # mask = (np.sum(mask, -1, keepdims=True) >= 1)
        mask = masks[:, :, 3]
        mask.reshape(mask.shape[0], mask.shape[1], 1)
        splash = np.where(mask, 255, 0).astype(np.uint8)
    else:
        splash = gray.astype(np.uint8)
    return splash


def detect_object_by_path(model, image_path=None):

    # Run model detection and generate the color splash effect
    print("Running on {}".format(args.image))

    # Read image
    image = skimage.io.imread(args.image)

    sttime = time.time()

    # Detect objects
    r = model.detect([image], verbose=0)[0]

    edtime = time.time()

    print("-----------------------------------------------------")
    print("WorkingTime: {} sec".format(edtime-sttime))

    # Color splash
    splash = get_mask(image, r['masks'], 0)

    # Save output
    file_name = "splash_{:%Y%m%dT%H%M%S}.png".format(datetime.datetime.now())
    skimage.io.imsave(file_name, splash)
    print("Saved to ", file_name)


def detect_object_by_data(model, image_data):

    sttime = time.time()

    # Detect objects
    r = model.detect([image_data], verbose=0)[0]

    edtime = time.time()

    print("-----------------------------------------------------")
    print("WorkingTime: {} sec".format(edtime-sttime))
    print(r['masks'].shape[-1])

    # # Color splash
    # splash = get_mask(image, r['masks'], 0)

    # # Save output
    # file_name = "splash_{:%Y%m%dT%H%M%S}.png".format(datetime.datetime.now())
    # skimage.io.imsave(file_name, splash)
    # print("Saved to ", file_name)


############################################################
#  Training
############################################################
if __name__ == '__main__':
    import argparse

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='infer Mask R-CNN to detect an object.')
    parser.add_argument('--weights', required=True,
                        metavar="/path/to/weights.h5",
                        help="Path to weights .h5 file or 'coco'")
    parser.add_argument('--image', required=True,
                        metavar="path or URL to image",
                        help='Image to apply the color splash effect on')
    args = parser.parse_args()

    # print("Weights: ", args.weights)
    # print("Dataset: ", args.dataset)
    # print("Logs: ", args.logs)

    config = InferenceConfig()
    config.display()

    # create a model
    model = modellib.MaskRCNN(mode="inference", config=config,
                              model_dir=DEFAULT_LOGS_DIR)

    # select weights file to load
    weights_path = args.weights

    # load weights
    print("Loading weights ", weights_path)
    model.load_weights(weights_path, by_name=True)

    # Train or evaluate
    detect_object(model, image_path=args.image)
