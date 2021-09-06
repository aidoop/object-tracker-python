import cv2
import numpy as np
import sys
import math


from pyaidoop.calibration.calibhandeye_handeye import HandEyeCalibration
from pyaidoop.etc.hm_util import HMUtil
from applications.objtracking.objecttracking_base import ObjectTracker

from dl.odapi.detector import ObjectDetector


# tensorflow object detection api(ODApi) object
class ODApiObject:
    def __init__(self, markerID, pivotOffset):
        self.markerID = markerID
        self.pivotOffset = pivotOffset
        self.targetPos = None


# tensorflow object detection api(ODApi) object tracker
class ODApiObjectTracker(ObjectTracker):

    # a best candidate...
    MODEL_PATH = "/home/jinwon/Documents/github/pyaidoop-pickpoint-finder/exported_model/ssd_mobilenet_v2_kimchi_1/saved_model/"
    LABELMAP_PATH = "/home/jinwon/Documents/github/pyaidoop-pickpoint-finder/models/sample_labelmap.pbtxt"
    DETECTION_THRESHOLD = 0.7  # 0.5

    def __init__(self):
        self.markerObjectList = []
        self.markerObjIDList = []

        # maskrcnn addition data
        self.center_point_list = []
        self.scores_list = []
        self.box_list = []

        # pose information
        self.mask_rect_list = []
        self.mask_angle_list = []

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

    # set detectable features and return the 2D or 3D positons in case that objects are detected..
    def find_tracking_object(self, *args):
        color_image = args[0]
        vtc = args[1]
        gripperOffset = args[2]
        # prepare list to give over the result objects
        resultList = list()

        ############################################################
        # detect objects
        if self.detector is not None:
            box_list = self.detector.detect_for_image(color_image)
            self.box_list = box_list
            print(self.box_list)
            # # how to parse box_list
            # for idx in range(len(boxes_list)):
            #     x_min = boxes_list[idx][0]
            #     y_min = boxes_list[idx][1]
            #     x_max = boxes_list[idx][2]
            #     y_max = boxes_list[idx][3]
            #     cls = str(boxes_list[idx][4])
            #     score = str(np.round(boxes_list[idx][-1], 2))

            # # get the minArearect and angle for each mask
            # (self.mask_rect_list, self.mask_angle_list) = self.estimate_pose(mask_list)
            # print(self.mask_rect_list)
            # print(self.mask_angle_list)

            # self.center_point_list = self.find_box_center_list(self.mask_rect_list)
            # print(self.center_point_list)

            # self.scores_list = self.maskdetect.get_scores()
            # assert self.scores_list is not None or self.center_point_list is not None

            # for idx, (markerObject, cpoint, angle) in enumerate(
            #     zip(self.markerObjectList, self.center_point_list, self.mask_angle_list)
            # ):
            #     tvec = vtc.vcap.get_3D_pos(cpoint[0], cpoint[1])
            #     # print("---------------------------------------------")
            #     # print(vtc.vcap.get_3D_pos(cpoint[0], cpoint[1]))
            #     # print("---------------------------------------------")
            #     # NOTE: don't need to consider rotation here. we don't have any pose inforation except translation
            #     rvec = np.array([0.0, 0.0, 0.0])

            #     # change a rotation vector to a rotation matrix
            #     rotMatrix = np.zeros(shape=(3, 3))

            #     cv2.Rodrigues(rvec, rotMatrix)

            #     # make a homogeneous matrix using a rotation matrix and a translation matrix
            #     hmCal2Cam = HMUtil.create_homogeneous_matrix(rotMatrix, tvec)

            #     # fix z + 0.01 regardless of some input offsets like tool offset, poi offset,...
            #     # hmWanted = HMUtil.convert_xyzabc_to_hm_by_deg(
            #     #     offsetPoint) if offsetPoint is not None else np.eye(4)
            #     # hmInput = np.dot(hmCal2Cam, hmWanted)

            #     # get a final position
            #     hmResult = np.dot(self.handEyeMat, hmCal2Cam)
            #     xyzuvw_list = HMUtil.convert_hm_to_xyzabc_by_deg(hmResult)
            #     xyzuvw_arr = np.array(xyzuvw_list)

            #     offsetPoint = (
            #         markerObject.pivotOffset + vtc.camObjOffset
            #         if vtc.camObjOffset is not None
            #         else markerObject.pivotOffset
            #     )

            #     xyzuvw_arr += offsetPoint.tolist()
            #     xyzuvw = xyzuvw_arr.tolist()

            #     # this conversion is moved to Operato, but come back by making robot move policy consistent
            #     if xyzuvw != None:
            #         [x, y, z, u, v, w] = xyzuvw
            #         # indy7 base position to gripper position
            #         # xyzuvw = [x, y, z, u*(-1), v+180.0, w]
            #         # NOTE: using the fixed rotation information
            #         # TODO: adjust 'w' value by input angle value
            #         angle = angle + 180.0 if angle <= -90.0 else angle  # TEST
            #         xyzuvw = [x, y, z, 180.0, 0.0, 180.0 - angle]

            #     # set final target position..
            #     markerObject.targetPos = xyzuvw

            #     # append this object to the list to be returned
            #     resultList.append(markerObject)
        else:
            pass

        return resultList

    def get_scores_list(self):
        return self.scores_list

    def get_boxed_image(self, input_image):
        return self.detector.get_bboxes_on_image(input_image, self.box_list)

    def estimate_pose(self, mask_list):
        box_list = list()
        angle_list = list()

        if len(mask_list) > 0:
            for mask in mask_list:
                mask_image = np.where(mask, 255, 0).astype(np.uint8)
                contours, hierarchy = cv2.findContours(
                    mask_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
                )

                if len(contours) > 0:
                    for contour in contours:
                        # get minimum-area rect of each contour
                        areaRect = cv2.minAreaRect(contour)

                        # get rotation angle and push into the list
                        (rectWidth, rectHeight) = areaRect[1]
                        angle = (
                            (areaRect[2] - 90)
                            if (rectWidth < rectHeight)
                            else areaRect[2]
                        )
                        angle_list.append(angle)

                        # get the rectangle coordinates of RotateRect
                        box = cv2.boxPoints(areaRect)

                        # push into the list
                        box_list.append(box)

                else:
                    print("find mask but can not get any contour", file=sys.stderr)
        else:
            print("find mask in mask_list", file=sys.stderr)

        return (box_list, angle_list)

    def find_box_center_list(self, rect_list):
        # assert self.maskdetect is not None

        # center_point_list = []

        # for mask_rect in self.mask_rect_list:
        #     mask_rect = np.int0(mask_rect)

        #     # get the points of two lines
        #     xx1 = np.int0((mask_rect[0][0] + mask_rect[1][0]) / 2)
        #     yy1 = np.int0((mask_rect[0][1] + mask_rect[1][1]) / 2)
        #     xx2 = np.int0((mask_rect[2][0] + mask_rect[3][0]) / 2)
        #     yy2 = np.int0((mask_rect[2][1] + mask_rect[3][1]) / 2)

        #     xx3 = np.int0((mask_rect[0][0] + mask_rect[3][0]) / 2)
        #     yy3 = np.int0((mask_rect[0][1] + mask_rect[3][1]) / 2)
        #     xx4 = np.int0((mask_rect[2][0] + mask_rect[1][0]) / 2)
        #     yy4 = np.int0((mask_rect[2][1] + mask_rect[1][1]) / 2)

        #     (cx, cy) = get_line_cross(xx1, yy1, xx2, yy2, xx3, yy3, xx4, yy4)
        #     center_point_list.append((np.int0(cx), np.int0(cy)))

        # return center_point_list
        return None
