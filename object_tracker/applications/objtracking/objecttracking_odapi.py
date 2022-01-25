import cv2
import numpy as np
import sys
import math
import collections


from pyaidoop.calibration.calibhandeye_handeye import HandEyeCalibration
from pyaidoop.etc.hm_util import HMUtil
from applications.objtracking.objecttracking_base import ObjectTracker

from dl.odapi.detector import ObjectDetector

from applications.config.appconfig import AppConfig


# tensorflow object detection api(ODApi) object
class ODApiObject:
    def __init__(self, markerID, pivotOffset):
        self.markerID = markerID
        self.pivotOffset = pivotOffset
        self.targetPos = None


# tensorflow object detection api(ODApi) object tracker
class ODApiObjectTracker(ObjectTracker):

    # a best candidate...
    MODEL_PATH = "/home/jinwon/Documents/github/pyaidoop-pickpoint-finder/exported_model/ssd_mobilenet_v2_pack_1/saved_model/"
    LABELMAP_PATH = "/home/jinwon/Documents/github/pyaidoop-pickpoint-finder/models/pack_labelmap.pbtxt"

    # detecton probability threshold
    DETECTION_THRESHOLD = 0.8  # 0.5

    # maximun depth value
    CRITERIA_DEPTH = (0.1, 0.3)

    # esimated result list count
    ESTIMATED_RESULT_COUNT = 10

    def __init__(self):
        # the list for input mark objects
        self.markerObjectList = []
        self.markerObjIDList = []

        # object addition data
        self.detected_objects = list()
        self.final_object = list()

        self.scores_list = []
        self.box_list = []

        # pose information
        self.object_center_list = []
        self.object_angle_list = []

        # detected line info.
        self.max_detected_line = None
        self.max_detected_line_length = 0
        self.max_detected_line_angle = 0

        # result collection
        self.queue_estimated_result = collections.deque(
            maxlen=self.ESTIMATED_RESULT_COUNT
        )

    # initialize parameters for any camera operation

    def initialize(self, *args):
        self.handEyeMat = args[4]  # HandEyeCalibration.loadTransformMatrix()
        self.markerObjectList.clear()
        self.markerObjIDList.clear()

        self.detector = ObjectDetector(
            self.MODEL_PATH,
            self.LABELMAP_PATH,
            class_id=None,
            threshold=self.DETECTION_THRESHOLD,
        )

    def set_model_path(self, model_path, labelmap_path):
        self.MODEL_PATH = model_path
        self.LABELMAP_PATH = labelmap_path

    def set_detection_threshold(self, threshold):
        self.DETECTION_THRESHOLD = threshold

    # set detectable features like marker id of aruco marker
    def set_tracking_object(self, object):
        assert isinstance(object, ODApiObject)

        # TODO: handles two more object here?
        self.markerObjIDList.append(object.markerID)
        self.markerObjectList.append(object)

    def get_tracking_object_list(self):
        return self.markerObjectList

    def get_tracking_object_id_list(self):
        return self.markerObjIDList

    def get_lines_from_image(self, input_image):
        input_gray_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

        edge_image = cv2.Canny(input_gray_image, 75, 150)
        lines = cv2.HoughLinesP(edge_image, 1, np.pi / 180, 30, maxLineGap=300)
        return lines

    def find_optimal_line(self, lines, depth_image):
        line_depth_averages = list()
        min_line_depth_average = 9999999  # set an enougth big value
        if len(lines) > 0:
            for index, value in enumerate(lines):
                x1, y1, x2, y2 = value[0]
                line_iter = self.create_line_iterator((x1, y1), (x2, y2), depth_image)

                line_depth_average = 0
                line_depth_average_index = 0
                for line_point in line_iter:
                    if line_point[2] != 0:
                        line_depth_average += line_point[2]
                        line_depth_average_index += 1
                    # else:
                    #     line_depth_average_index = 0
                    #     break

                if line_depth_average_index > 0:
                    line_depth_average = line_depth_average / line_depth_average_index

                    # check the length of detected line is longer than minimun value either width or height of the extracted imate
                    min_line_length = (
                        int(min(depth_image.shape[0], depth_image.shape[1])) / 2
                    )
                    if line_depth_average_index > min_line_length:
                        line_depth_averages.append(line_depth_average)
                    else:
                        line_depth_averages.append(min_line_depth_average)
                else:
                    line_depth_averages.append(min_line_depth_average)

            min_value = min(line_depth_averages)
            min_index = line_depth_averages.index(min_value)

        detected_line = lines[min_index] if len(lines) > 0 else None
        return detected_line

    def find_best_box(self, box_list):
        if len(box_list) > 0:
            best_box = box_list[0]
            for idx in range(len(box_list)):
                score = np.round(box_list[idx][-1], 3)

    def determine_best_pick(self, queue_estimated_result):
        if len(queue_estimated_result) < self.ESTIMATED_RESULT_COUNT:
            return None

        queue_estimated_list = list(queue_estimated_result)
        queue_estimated_list.sort(key=lambda x: x[2])

        return queue_estimated_list[5]

    def get_box_iou(self, boxListA: list, boxListB: list) -> float:
        if len(boxListA) == 0 or len(boxListB) == 0:
            return 0.0

        # get the first box all the time
        boxA = boxListA[0]
        boxB = boxListB[0]

        # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])

        # compute the area of intersection rectangle
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)

        # compute the area of both the prediction and ground-truth
        # rectangles
        boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)

        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = interArea / float(boxAArea + boxBArea - interArea)

        # return the intersection over union value
        return iou

    # initialize the maximum length of detected line
    def initialize_maximum_detected_line(self):
        self.max_detected_line = None
        self.max_detected_line_length = 0

    # set detectable features and return the 2D or 3D positons in case that objects are detected..
    def find_tracking_object(self, *args):
        color_image = args[0]
        vtc = args[1]
        gripperOffset = args[2]
        depth_image = args[3]
        # prepare list to give over the result objects
        resultList = list()

        self.detected_objects.clear()

        ############################################################
        # detect objects
        if self.detector is not None:
            box_list = self.detector.detect_for_image(color_image)

            for idx in range(len(box_list)):
                x_min = box_list[idx][0]
                y_min = box_list[idx][1]
                x_max = box_list[idx][2]
                y_max = box_list[idx][3]
                cls = str(box_list[idx][4])
                score = str(np.round(box_list[idx][-1], 2))

                object_image = color_image[y_min:y_max, x_min:x_max]
                object_depth_image = depth_image[y_min:y_max, x_min:x_max]

                # find the lines in this image
                lines = self.get_lines_from_image(object_image)

                # find a line with minium average depth value in the lines of the image
                detected_line = self.find_optimal_line(lines, object_depth_image)
                x1_det, y1_det, x2_det, y2_det = detected_line[0]
                detected_line_length = math.sqrt(
                    (x1_det - x2_det) * (x1_det - x2_det)
                    + (y1_det - y2_det) * (y1_det - y2_det)
                )

                # check the availability of the detected line
                if detected_line_length > self.max_detected_line_length:
                    self.max_detected_line = detected_line
                    self.max_detected_line_length = detected_line_length

                # set the optimal line to member list variable
                if self.max_detected_line is not None:
                    x1_det, y1_det, x2_det, y2_det = self.max_detected_line[0]
                    self.detected_objects.append(
                        {
                            "line": (
                                x1_det + x_min,
                                y1_det + y_min,
                                x2_det + x_min,
                                y2_det + y_min,
                            ),
                            "class": cls,
                            "score": score,
                        }
                    )

            # get IOU value of two boxes.
            if AppConfig.EnableBoxIOU == True:
                box_iou = self.get_box_iou(box_list, self.box_list)
                if box_iou < AppConfig.BoxIOUCriteria:
                    self.initialize_maximum_detected_line()

            # if detected object cannot be found, initialize saved detected line(the maximum length)
            if len(self.detected_objects) == 0:
                self.initialize_maximum_detected_line()

            # set internal box list
            self.box_list = box_list

            # TODO: need to debug this but be commented later
            # print(self.detected_objects)

            (
                self.object_center_list,
                self.object_angle_list,
                self.scores_list,
            ) = self.estimate_pose(self.detected_objects)

            for idx, (markerObject, object_center, object_angle,) in enumerate(
                zip(
                    self.markerObjectList,
                    self.object_center_list,
                    self.object_angle_list,
                )
            ):
                # process coord. translation & send it to the server in case with first index only
                if idx == 0:
                    # if abs(self.max_detected_line_angle - object_angle) > 30:
                    #     self.max_detected_line = None
                    #     self.max_detected_line_length = 0
                    # self.max_detected_line_angle = object_angle

                    try:

                        tvec = vtc.vcap.get_3D_pos(object_center[0], object_center[1])
                        # print("---------------------------------------------")
                        # print(vtc.vcap.get_3D_pos(cpoint[0], cpoint[1]))
                        # print("---------------------------------------------")
                        # NOTE: don't need to consider rotation here. we don't have any pose inforation except translation
                        rvec = np.array([0.0, 0.0, 0.0])

                        # change a rotation vector to a rotation matrix
                        rotMatrix = np.zeros(shape=(3, 3))

                        cv2.Rodrigues(rvec, rotMatrix)

                        # make a homogeneous matrix using a rotation matrix and a translation matrix
                        hmCal2Cam = HMUtil.create_homogeneous_matrix(rotMatrix, tvec)

                        # fix z + 0.01 regardless of some input offsets like tool offset, poi offset,...
                        # hmWanted = HMUtil.convert_xyzabc_to_hm_by_deg(
                        #     offsetPoint) if offsetPoint is not None else np.eye(4)
                        # hmInput = np.dot(hmCal2Cam, hmWanted)

                        # get a final position
                        hmResult = np.dot(self.handEyeMat, hmCal2Cam)
                        xyzuvw_list = HMUtil.convert_hm_to_xyzabc_by_deg(hmResult)
                        xyzuvw_arr = np.array(xyzuvw_list)

                        offsetPoint = (
                            markerObject.pivotOffset + vtc.camObjOffset
                            if vtc.camObjOffset is not None
                            else markerObject.pivotOffset
                        )

                        xyzuvw_arr += offsetPoint.tolist()
                        xyzuvw = xyzuvw_arr.tolist()

                        # this conversion is moved to Operato, but come back by making robot move policy consistent
                        if xyzuvw != None:
                            [x, y, z, u, v, w] = xyzuvw

                            if (
                                z > self.CRITERIA_DEPTH[0]
                                and z < self.CRITERIA_DEPTH[1]
                            ):
                                # indy7 base position to gripper position
                                # xyzuvw = [x, y, z, u*(-1), v+180.0, w]
                                # NOTE: using the fixed rotation information
                                # TODO: adjust 'w' value by input angle value
                                object_angle = (
                                    object_angle + 180.0
                                    if object_angle <= -90.0
                                    else object_angle
                                )  # TEST
                                xyzuvw = [x, y, z, 180.0, 0.0, 180.0 - object_angle]

                                # result queing
                                self.queue_estimated_result.append(xyzuvw)
                                determined_xyzuvw = self.determine_best_pick(
                                    self.queue_estimated_result
                                )

                                # set final target position..
                                markerObject.targetPos = determined_xyzuvw

                                # append this object to the list to be returned
                                resultList.append(markerObject)
                    except Exception as ex:
                        resultList = list()

        else:
            pass

        return resultList

    def get_scores_list(self):
        return self.scores_list

    def get_boxed_image(self, input_image):
        return self.detector.get_bboxes_on_image(input_image, self.box_list)

    def get_pickpoint_image(self, input_image):
        boxed_image = self.detector.get_bboxes_on_image(input_image, self.box_list)

        # draw a object box

        for idx, (detected_object, object_angle) in enumerate(
            zip(self.detected_objects, self.object_angle_list)
        ):
            x1 = detected_object["line"][0]
            y1 = detected_object["line"][1]
            x2 = detected_object["line"][2]
            y2 = detected_object["line"][3]
            boxed_image = cv2.line(boxed_image, (x1, y1), (x2, y2), (0, 0, 128), 3)

            # draw an score information
            score = str(np.round(self.box_list[idx][-1], 2))
            score_text = f"Score: {score}"
            score_text_size, _ = cv2.getTextSize(
                score_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2
            )
            cv2.putText(
                boxed_image,
                score_text,
                (
                    self.box_list[idx][2] + 3,
                    self.box_list[idx][1] + score_text_size[1],
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

            # draw an angle information
            angle_text = f"Angle: {object_angle:.2f}"
            angle_text_size, _ = cv2.getTextSize(
                angle_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2
            )
            cv2.putText(
                boxed_image,
                angle_text,
                (
                    self.box_list[idx][2] + 3,
                    self.box_list[idx][1] + angle_text_size[1] + score_text_size[1] + 3,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

        # draw a picking point
        for center_point in self.object_center_list:
            cv2.drawMarker(
                boxed_image,
                center_point,
                (200, 30, 30),
                markerType=cv2.MARKER_TRIANGLE_UP,
                markerSize=20,
                thickness=3,
            )

        # draw text information
        object_count_text = f"Object Count: {len(self.detected_objects)}"
        text_size, _ = cv2.getTextSize(
            object_count_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2
        )
        cv2.putText(
            boxed_image,
            object_count_text,
            (3, text_size[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
        )

        return boxed_image

    def estimate_pose(self, object_list):
        center_list = list()
        angle_list = list()
        score_list = list()

        if len(object_list) > 0:
            for object_one in object_list:
                # get a center point
                p1_x = object_one["line"][0]
                p1_y = object_one["line"][1]
                p2_x = object_one["line"][2]
                p2_y = object_one["line"][3]
                center_list.append((int((p1_x + p2_x) / 2), int((p1_y + p2_y) / 2)))

                # get a angle
                angle = math.atan2(p1_y - p2_y, p1_x - p2_x)
                angle = angle * 180 / math.pi
                angle_list.append(angle)

                # get a score
                score = object_one["score"]
                score_list.append(score)
        else:
            print("can't find any object in object_list", file=sys.stderr)

        return (center_list, angle_list, score_list)

    def create_line_iterator(self, P1, P2, img):
        """
        Produces and array that consists of the coordinates and intensities of each pixel in a line between two points

        Parameters:
            -P1: a numpy array that consists of the coordinate of the first point (x,y)
            -P2: a numpy array that consists of the coordinate of the second point (x,y)
            -img: the image being processed

        Returns:
            -it: a numpy array that consists of the coordinates and intensities of each pixel in the radii (shape: [numPixels, 3], row = [x,y,intensity])
        """
        # define local variables for readability
        imageH = img.shape[0]
        imageW = img.shape[1]
        P1X = P1[0]
        P1Y = P1[1]
        P2X = P2[0]
        P2Y = P2[1]

        # difference and absolute difference between points
        # used to calculate slope and relative location between points
        dX = P2X - P1X
        dY = P2Y - P1Y
        dXa = np.abs(dX)
        dYa = np.abs(dY)

        # predefine numpy array for output based on distance between points
        itbuffer = np.empty(shape=(np.maximum(dYa, dXa), 3), dtype=np.float32)
        itbuffer.fill(np.nan)

        # Obtain coordinates along the line using a form of Bresenham's algorithm
        negY = P1Y > P2Y
        negX = P1X > P2X
        if P1X == P2X:  # vertical line segment
            itbuffer[:, 0] = P1X
            if negY:
                itbuffer[:, 1] = np.arange(P1Y - 1, P1Y - dYa - 1, -1)
            else:
                itbuffer[:, 1] = np.arange(P1Y + 1, P1Y + dYa + 1)
        elif P1Y == P2Y:  # horizontal line segment
            itbuffer[:, 1] = P1Y
            if negX:
                itbuffer[:, 0] = np.arange(P1X - 1, P1X - dXa - 1, -1)
            else:
                itbuffer[:, 0] = np.arange(P1X + 1, P1X + dXa + 1)
        else:  # diagonal line segment
            steepSlope = dYa > dXa
            if steepSlope:
                slope = dX.astype(np.float32) / dY.astype(np.float32)
                if negY:
                    itbuffer[:, 1] = np.arange(P1Y - 1, P1Y - dYa - 1, -1)
                else:
                    itbuffer[:, 1] = np.arange(P1Y + 1, P1Y + dYa + 1)
                itbuffer[:, 0] = (slope * (itbuffer[:, 1] - P1Y)).astype(np.int) + P1X
            else:
                slope = dY.astype(np.float32) / dX.astype(np.float32)
                if negX:
                    itbuffer[:, 0] = np.arange(P1X - 1, P1X - dXa - 1, -1)
                else:
                    itbuffer[:, 0] = np.arange(P1X + 1, P1X + dXa + 1)
                itbuffer[:, 1] = (slope * (itbuffer[:, 0] - P1X)).astype(np.int) + P1Y

        # Remove points outside of image
        colX = itbuffer[:, 0]
        colY = itbuffer[:, 1]
        itbuffer = itbuffer[
            (colX >= 0) & (colY >= 0) & (colX < imageW) & (colY < imageH)
        ]

        # Get intensities from img ndarray
        itbuffer[:, 2] = img[
            itbuffer[:, 1].astype(np.uint), itbuffer[:, 0].astype(np.uint)
        ]

        return itbuffer
