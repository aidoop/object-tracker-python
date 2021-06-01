import cv2
import numpy as np
import tensorflow as tf
from PIL import Image

import time

from aidoop.yolo.inference import YoloDetect
import aidoop.yolo.utils as utils


if __name__ == "__main__":
    input_image_path = "/home/jinwon/Documents/github/factory/packages/aidoop-r/object-tracker-python/examples/yolo-cabbage/samples/cabbage.jpg"

    yolo_detect = YoloDetect(
        "tf",
        "/home/jinwon/Documents/github/factory/packages/aidoop-r/object-tracker-python/examples/yolo-cabbage/yolov4-cusotm",
    )

    for i in range(30):
        result = yolo_detect.detect_object_by_path(input_image_path)

    print(result)

    original_image = cv2.imread(input_image_path)
    original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)

    display_image = utils.draw_bbox(original_image, result)
    # display_image = utils.draw_bbox(image_data*255, pred_bbox)
    display_image = Image.fromarray(display_image.astype(np.uint8))
    display_image.show()