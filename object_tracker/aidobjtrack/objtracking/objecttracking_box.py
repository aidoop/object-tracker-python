import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import os
import math

from aidobjtrack.abc.objecttracking_base import ObjectTracker
from aidobjtrack.handeye.calibhandeye_handeye import HandEyeCalibration
from aidobjtrack.util.hm_util import HMUtil

# mask rcnn detector
from mrcnn import inference as mo


def get_line_length(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2+(y2-y1)**2)


def get_line_cross(x11, y11, x12, y12, x21, y21, x22, y22):
    if x12 == x11 or x22 == x21:
        if x12 == x11:
            cx = x12
            m2 = (y22 - y21) / (x22 - x21)
            cy = m2 * (cx - x21) + y21
            return cx, cy
        if x22 == x21:
            cx = x22
            m1 = (y12 - y11) / (x12 - x11)
            cy = m1 * (cx - x11) + y11
            return cx, cy

    m1 = (y12 - y11) / (x12 - x11)
    m2 = (y22 - y21) / (x22 - x21)
    if m1 == m2:
        return None

    cx = (x11 * m1 - y11 - x21 * m2 + y21) / (m1 - m2)
    cy = m1 * (cx - x11) + y11

    return cx, cy


class BoxObject:
    def __init__(self, markerID, pivotOffset):
        self.markerID = markerID
        self.pivotOffset = pivotOffset
        self.targetPos = None


class BoxObjectTracker(ObjectTracker):
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201017T1422/mask_rcnn_object-train_0043.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201017T1713/mask_rcnn_object-train_0094.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201017T2218/mask_rcnn_object-train_0150.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201018T1104/mask_rcnn_object-train_0200.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201018T2110/mask_rcnn_object-train_0100.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201019T1829/mask_rcnn_object-train_0132.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201022T0658/mask_rcnn_object-train_0080.h5" # bad
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201022T2147/mask_rcnn_object-train_0089.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201023T0653/mask_rcnn_object-train_0050.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201023T1430/mask_rcnn_object-train_0093.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201024T0922/mask_rcnn_object-train_0098.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201024T2354/mask_rcnn_object-train_0073.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201025T1253/mask_rcnn_object-train_0099.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201030T1653/mask_rcnn_object-train_0188.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201030T1653/mask_rcnn_object-train_0137.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201031T0859/mask_rcnn_object-train_0120.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201031T1858/mask_rcnn_object-train_0180.h5"

    # a candidate <-- [base: DelBox2_2] 단일 박스는 잘 인식되는 것으로 보임. 박스의 종류를 늘리면 추가적인 박스에도 적응할 것으로 예상.
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs/object-train20201020T1933/mask_rcnn_object-train_0179.h5"

    # a valualbe candidate < [base: DelBox8] Box Dataset을 대폭 추가(thanks to NamYW & his friend)
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201101T1825/mask_rcnn_object-train_0098.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201101T1825/mask_rcnn_object-train_0178.h5"

    # a best candidate...
    OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201101T1825/mask_rcnn_object-train_0098.h5"

    # resnet-50 test
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201122T1517/mask_rcnn_object-train_0059.h5"

    LOGS_PATH = "/home/jinwon/Documents/github/object-tracker-python-data/logs"

    def __init__(self):
        self.markerObjectList = []
        self.markerObjIDList = []

        # maskrcnn addition data
        self.center_point_list = []
        self.scores_list = []
        self.mask_list = []

        # pose information
        self.mask_rect_list = []
        self.mask_angle_list = []

    # initialize parameters for any camera operation

    def initialize(self, *args):
        self.handEyeMat = args[4]  # HandEyeCalibration.loadTransformMatrix()
        self.markerObjectList.clear()
        self.markerObjIDList.clear()

        self.maskdetect = mo.MaskRcnnDetect(
            BoxObjectTracker.OBJECT_WEIGHT_PATH, BoxObjectTracker.LOGS_PATH)

    # set detectable features like marker id of aruco marker
    def setTrackingObject(self, object):
        assert(isinstance(object, BoxObject))

        # TODO: handles two more object here?
        self.markerObjIDList.append(object.markerID)
        self.markerObjectList.append(object)

    def getTrackingObjectList(self):
        return self.markerObjectList

    def getTrackingObjIDList(self):
        return self.markerObjIDList

    # set detectable features and return the 2D or 3D positons in case that objects are detected..
    def findObjects(self, *args):
        color_image = args[0]
        vtc = args[1]
        gripperOffset = args[2]
        # prepare list to give over the result objects
        resultList = list()

        ############################################################
        # detect objects
        if self.maskdetect is not None:
            mask_list = self.maskdetect.detect_object_by_data(
                color_image)
            self.mask_list = mask_list

            # get the minArearect and angle for each mask
            (self.mask_rect_list, self.mask_angle_list) = self.estimatePose(mask_list)
            print(self.mask_rect_list)
            print(self.mask_angle_list)

            self.center_point_list = self.findBoxCenterList(
                self.mask_rect_list)
            print(self.center_point_list)

            # self.center_point_list = self.maskdetect.get_center_points(
            #     mask_list)
            # print('center point: ', self.center_point_list)

            self.scores_list = self.maskdetect.get_scores()
            assert self.scores_list is not None or self.center_point_list is not None

            for idx, (markerObject, cpoint, angle) in enumerate(zip(self.markerObjectList, self.center_point_list, self.mask_angle_list)):
                tvec = vtc.vcap.get_3D_pos(cpoint[0], cpoint[1])
                # print("---------------------------------------------")
                # print(vtc.vcap.get_3D_pos(cpoint[0], cpoint[1]))
                # print("---------------------------------------------")
                # NOTE: don't need to consider rotation here. we don't have any pose inforation except translation
                rvec = np.array([0.0, 0.0, 0.0])

                # change a rotation vector to a rotation matrix
                rotMatrix = np.zeros(shape=(3, 3))

                cv2.Rodrigues(rvec, rotMatrix)

                # make a homogeneous matrix using a rotation matrix and a translation matrix
                hmCal2Cam = HMUtil.makeHM(rotMatrix, tvec)

                # fix z + 0.01 regardless of some input offsets like tool offset, poi offset,...
                # hmWanted = HMUtil.convertXYZABCtoHMDeg(
                #     offsetPoint) if offsetPoint is not None else np.eye(4)
                # hmInput = np.dot(hmCal2Cam, hmWanted)

                # get a final position
                hmResult = np.dot(self.handEyeMat, hmCal2Cam)
                xyzuvw_list = HMUtil.convertHMtoXYZABCDeg(hmResult)
                xyzuvw_arr = np.array(xyzuvw_list)

                offsetPoint = markerObject.pivotOffset + \
                    vtc.camObjOffset if vtc.camObjOffset is not None else markerObject.pivotOffset

                xyzuvw_arr += offsetPoint.tolist()
                xyzuvw = xyzuvw_arr.tolist()

                # this conversion is moved to Operato, but come back by making robot move policy consistent
                if xyzuvw != None:
                    [x, y, z, u, v, w] = xyzuvw
                    # indy7 base position to gripper position
                    # xyzuvw = [x, y, z, u*(-1), v+180.0, w]
                    # NOTE: using the fixed rotation information
                    # TODO: adjust 'w' value by input angle value
                    angle = angle + 180.0 if angle <= -90.0 else angle     # TEST
                    xyzuvw = [x, y, z, 180.0, 0.0, 180.0-angle]

                # set final target position..
                markerObject.targetPos = xyzuvw

                # append this object to the list to be returned
                resultList.append(markerObject)
        else:
            pass

        return resultList

    def getMaskImage(self, color_image, width, height):
        assert self.maskdetect is not None

        # mask_list = self.maskdetect.detect_object_by_data(color_image)
        # get mask_list
        mask_list = self.mask_list

        # get mask image
        mask_gray_image = self.maskdetect.get_mask_image(
            mask_list, width, height)
        mask_image = cv2.cvtColor(mask_gray_image, cv2.COLOR_GRAY2RGB)

        for mask_rect in self.mask_rect_list:
            mask_rect = np.int0(mask_rect)

            # draw box
            cv2.drawContours(mask_image, [mask_rect], 0, (255, 0, 0), 2)

            # draw center line
            xx1 = np.int0((mask_rect[0][0] + mask_rect[1][0])/2)
            yy1 = np.int0((mask_rect[0][1] + mask_rect[1][1])/2)
            xx2 = np.int0((mask_rect[2][0] + mask_rect[3][0])/2)
            yy2 = np.int0((mask_rect[2][1] + mask_rect[3][1])/2)

            len1 = get_line_length(xx1, yy1, xx2, yy2)

            xx3 = np.int0((mask_rect[0][0] + mask_rect[3][0])/2)
            yy3 = np.int0((mask_rect[0][1] + mask_rect[3][1])/2)
            xx4 = np.int0((mask_rect[2][0] + mask_rect[1][0])/2)
            yy4 = np.int0((mask_rect[2][1] + mask_rect[1][1])/2)

            len2 = get_line_length(xx3, yy3, xx4, yy4)

            if(len1 >= len2):
                color1 = (0, 0, 255)
                color2 = (0, 255, 0)
            else:
                color1 = (0, 255, 0)
                color2 = (0, 0, 255)

            cv2.line(mask_image, (xx1, yy1), (xx2, yy2), color1, 2)
            cv2.line(mask_image, (xx3, yy3), (xx4, yy4), color2, 2)

        return mask_image

    def putTextData(self, color_image):
        assert color_image is not None

        # prepare font data
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        fontColor = (255, 255, 0)
        fontColor2 = (0, 0, 255)
        lineType = 1

        for idx, (cpoint, score, angle) in enumerate(zip(self.center_point_list, self.scores_list, self.mask_angle_list)):
            # print(idx, (cpoint, score))
            cv2.putText(color_image, '%0.8f' % score, cpoint, font,
                        fontScale, fontColor, lineType)
            (xx, yy) = cpoint
            cv2.putText(color_image, '%0.2f' % angle, (xx, yy+15), font,
                        fontScale, fontColor2, lineType)

    def getScoresList(self):
        return self.scores_list

    def estimatePose(self, mask_list):
        box_list = list()
        angle_list = list()

        if len(mask_list) > 0:
            for mask in mask_list:
                mask_image = np.where(mask, 255, 0).astype(np.uint8)
                contours, hierarchy = cv2.findContours(
                    mask_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours) > 0:
                    for contour in contours:
                        # get minimum-area rect of each contour
                        areaRect = cv2.minAreaRect(contour)

                        # get rotation angle and push into the list
                        (rectWidth, rectHeight) = areaRect[1]
                        angle = (areaRect[2] - 90) if(rectWidth <
                                                      rectHeight) else areaRect[2]
                        angle_list.append(angle)

                        # get the rectangle coordinates of RotateRect
                        box = cv2.boxPoints(areaRect)

                        # push into the list
                        box_list.append(box)

                else:
                    print('find mask but can not get any contour', file=sys.stderr)
        else:
            print('find mask in mask_list', file=sys.stderr)

        return (box_list, angle_list)

    def findBoxCenterList(self, rect_list):
        assert self.maskdetect is not None

        center_point_list = []

        for mask_rect in self.mask_rect_list:
            mask_rect = np.int0(mask_rect)

            # get the points of two lines
            xx1 = np.int0((mask_rect[0][0] + mask_rect[1][0])/2)
            yy1 = np.int0((mask_rect[0][1] + mask_rect[1][1])/2)
            xx2 = np.int0((mask_rect[2][0] + mask_rect[3][0])/2)
            yy2 = np.int0((mask_rect[2][1] + mask_rect[3][1])/2)

            xx3 = np.int0((mask_rect[0][0] + mask_rect[3][0])/2)
            yy3 = np.int0((mask_rect[0][1] + mask_rect[3][1])/2)
            xx4 = np.int0((mask_rect[2][0] + mask_rect[1][0])/2)
            yy4 = np.int0((mask_rect[2][1] + mask_rect[1][1])/2)

            (cx, cy) = get_line_cross(xx1, yy1, xx2, yy2, xx3, yy3, xx4, yy4)
            center_point_list.append((np.int0(cx), np.int0(cy)))

        return center_point_list
