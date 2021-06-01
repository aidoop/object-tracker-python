import cv2
import numpy as np
import tensorflow as tf
from PIL import Image

import time

from aidoop.yolo.inference import YoloDetect
import aidoop.yolo.utils as utils


if __name__ == "__main__":
    input_image_path = "/home/jinwon/Documents/github/factory/packages/aidoop-r/object-tracker-python/examples/yolo-cabbage/samples/cabbage3.jpg"

    yolo_detect = YoloDetect(
        "tf",
        "/home/jinwon/Documents/github/factory/packages/aidoop-r/object-tracker-python/examples/yolo-cabbage/yolov4-cusotm",
    )

    original_image = cv2.imread(input_image_path)
    total_elapsed_time = 0
    for i in range(31):
        start_time = time.time()
        result = yolo_detect.detect_object_by_data(original_image)
        if i > 0:
            diff_time = (time.time() - start_time) * 1000
            print(f"[{i:02d}] detection time(ms): {diff_time}")
            total_elapsed_time += diff_time

    print("-------------------------------------------------------")
    print(f"average detection time(ms): {total_elapsed_time / 30}")

    original_image = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)

    display_image = utils.draw_bbox(original_image, result)
    # display_image = utils.draw_bbox(image_data*255, pred_bbox)
    display_image = Image.fromarray(display_image.astype(np.uint8))
    display_image.show()