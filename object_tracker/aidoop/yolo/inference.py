import sys
import tensorflow as tf
import time

physical_devices = tf.config.experimental.list_physical_devices("GPU")
if len(physical_devices) > 0:
    tf.config.experimental.set_memory_growth(physical_devices[0], True)
from absl import app, flags, logging
from absl.flags import FLAGS
import aidoop.yolo.utils as utils
from aidoop.yolo.yolov4 import filter_boxes
from tensorflow.python.saved_model import tag_constants
from PIL import Image
import cv2
import numpy as np
from enum import Enum


from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

flags.DEFINE_string("framework", "tf", "(tf, tflite, trt")
flags.DEFINE_string("weights", "./checkpoints/yolov4-416", "path to weights file")
flags.DEFINE_integer("size", 416, "resize images to")
flags.DEFINE_boolean("tiny", False, "yolo or yolo-tiny")
flags.DEFINE_string("model", "yolov4", "yolov3 or yolov4")
flags.DEFINE_string("image", "./data/kite.jpg", "path to input image")
flags.DEFINE_string("output", "result.png", "path to output image")
flags.DEFINE_float("iou", 0.45, "iou threshold")
flags.DEFINE_float("score", 0.25, "score threshold")


class YoloDetect:
    FRAMEWORK_TYPE = "tf"
    WEIGHT_PATH = ""
    IMAGE_RESIZE = 416
    TINY = False
    YOLO_TYPE = "yolov4"
    IOU_THRESHOLD = 0.45
    CLASS_SCORE = 0.25

    def __init__(
        self,
        framework_type="tf",
        weight_file_path="",
        image_resize=416,
        yolo_type="yolov4",
    ):
        assert (framework_type is not None) and (
            weight_file_path is not None
        ), "framework type and weight_file_path should not be empty"
        self.framework_type = framework_type
        self.weight_file_path = weight_file_path
        self.image_resize = image_resize
        self.yolo_type = yolo_type

        self.boxes = None
        self.pred_conf = None

        if self.framework_type == "tflite":
            self.model = tf.lite.Interpreter(model_path=FLAGS.weights)
        else:
            self.model = tf.saved_model.load(
                self.weight_file_path, tags=[tag_constants.SERVING]
            )

    def process_detect(self, images_data):
        try:
            if self.framework_type == "tflite":
                self.model = tf.lite.Interpreter(model_path=FLAGS.weights)
                self.model.allocate_tensors()
                input_details = self.model.get_input_details()
                output_details = self.model.get_output_details()
                # print(input_details)
                # print(output_details)
                self.model.set_tensor(input_details[0]["index"], images_data)
                self.model.invoke()
                pred = [
                    self.model.get_tensor(output_details[i]["index"])
                    for i in range(len(output_details))
                ]
                if FLAGS.model == "yolov3" and FLAGS.tiny == True:
                    boxes, pred_conf = filter_boxes(
                        pred[1],
                        pred[0],
                        score_threshold=0.25,
                        input_shape=tf.constant([self.image_resize, self.image_resize]),
                    )
                else:
                    boxes, pred_conf = filter_boxes(
                        pred[0],
                        pred[1],
                        score_threshold=0.25,
                        input_shape=tf.constant([self.image_resize, self.image_resize]),
                    )
            else:
                batch_data = tf.constant(images_data)
                infer = self.model.signatures["serving_default"]

                pred_bbox = infer(batch_data)
                for key, value in pred_bbox.items():
                    boxes = value[:, :, 0:4]
                    pred_conf = value[:, :, 4:]
        except Exception as ex:
            print("Error :", ex, file=sys.stderr)
            boxes = pred_conf = None

        return (boxes, pred_conf)

    def postprocess(self, boxes, pred_conf):
        (
            boxes,
            scores,
            classes,
            valid_detections,
        ) = tf.image.combined_non_max_suppression(
            boxes=tf.reshape(boxes, (tf.shape(boxes)[0], -1, 1, 4)),
            scores=tf.reshape(
                pred_conf, (tf.shape(pred_conf)[0], -1, tf.shape(pred_conf)[-1])
            ),
            max_output_size_per_class=50,
            max_total_size=50,
            iou_threshold=YoloDetect.IOU_THRESHOLD,
            score_threshold=YoloDetect.CLASS_SCORE,
        )
        pred_bbox = [
            boxes.numpy(),
            scores.numpy(),
            classes.numpy(),
            valid_detections.numpy(),
        ]
        return pred_bbox

    def detect_object_by_path(self, image_path):
        image_input_data = cv2.imread(image_path)
        return self.detect_object_by_data(image_input_data)

    def detect_object_by_data(self, image_data):
        image_input_data = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)

        image_resize_data = cv2.resize(
            image_input_data, (self.image_resize, self.image_resize)
        )
        image_resize_data = image_resize_data / 255.0

        images_data = []
        for i in range(1):
            images_data.append(image_resize_data)
        images_data = np.asarray(images_data).astype(np.float32)

        # detect objects
        (self.boxes, self.pred_conf) = self.process_detect(images_data)

        pred_bbox = self.postprocess(self.boxes, self.pred_conf)

        return pred_bbox

    def get_box_list(self):
        return self.boxes

    def get_pred_conf_list(self):
        return self.pred_conf
