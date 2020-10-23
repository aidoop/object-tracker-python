import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import os

from aidobjtrack.abc.objecttracking_base import ObjectTracker
from aidobjtrack.handeye.calibhandeye_handeye import HandEyeCalibration
from aidobjtrack.util.hm_util import HMUtil

# mask rcnn detector
from mrcnn import inference as mo


class MrcnnObject:
    def __init__(self, markerID, pivotOffset):
        self.markerID = markerID
        self.pivotOffset = pivotOffset
        self.targetPos = None


class MrcnnObjectTracker(ObjectTracker):
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201017T1422/mask_rcnn_object-train_0043.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201017T1713/mask_rcnn_object-train_0094.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201017T2218/mask_rcnn_object-train_0150.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201018T1104/mask_rcnn_object-train_0200.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201018T2110/mask_rcnn_object-train_0100.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201019T1829/mask_rcnn_object-train_0132.h5"
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201022T0658/mask_rcnn_object-train_0080.h5" # bad
    # OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201022T2147/mask_rcnn_object-train_0089.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201022T2147/mask_rcnn_object-train_0089.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201023T0653/mask_rcnn_object-train_0050.h5"

    # a candidate <-- [base: DelBox2_2] 단일 박스는 잘 인식되는 것으로 보임. 박스의 종류를 늘리면 추가적인 박스에도 적응할 것으로 예상.
    OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201020T1933/mask_rcnn_object-train_0179.h5"

    LOGS_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs"

    def __init__(self):
        self.markerObjectList = []
        self.markerObjIDList = []

        # maskrcnn addition data
        self.center_point_list = []
        self.scores_list = []

    # initialize parameters for any camera operation
    def initialize(self, *args):
        self.handEyeMat = args[4]  # HandEyeCalibration.loadTransformMatrix()
        self.markerObjectList.clear()
        self.markerObjIDList.clear()

        self.maskdetect = mo.MaskRcnnDetect(
            MrcnnObjectTracker.OBJECT_WEIGHT_PATH, MrcnnObjectTracker.LOGS_PATH)

    # set detectable features like marker id of aruco marker
    def setTrackingObject(self, object):
        assert(isinstance(object, MrcnnObject))

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

            self.center_point_list = self.maskdetect.get_center_points(
                mask_list)
            print('center point: ', self.center_point_list)

            self.scores_list = self.maskdetect.get_scores()
            assert self.scores_list is not None or self.center_point_list is not None

            for idx, (markerObject, cpoint) in enumerate(zip(self.markerObjectList, self.center_point_list)):
                tvec = vtc.vcap.get3DPosition(cpoint[0], cpoint[1])
                # print("---------------------------------------------")
                # print(vtc.vcap.get3DPosition(cpoint[0], cpoint[1]))
                # print("---------------------------------------------")
                # NOTE: don't need to consider rotation here. we don't have any pose inforation except translation
                rvec = np.array([0.0, 0.0, 0.0])

                # change a rotation vector to a rotation matrix
                rotMatrix = np.zeros(shape=(3, 3))

                cv2.Rodrigues(rvec, rotMatrix)

                # make a homogeneous matrix using a rotation matrix and a translation matrix
                hmCal2Cam = HMUtil.makeHM(rotMatrix, tvec)

                offsetPoint = markerObject.pivotOffset + \
                    vtc.camObjOffset if vtc.camObjOffset is not None else markerObject.pivotOffset

                # fix z + 0.01 regardless of some input offsets like tool offset, poi offset,...
                hmWanted = HMUtil.convertXYZABCtoHMDeg(
                    offsetPoint) if offsetPoint is not None else np.eye(4)
                hmInput = np.dot(hmCal2Cam, hmWanted)

                # get a final position
                hmResult = np.dot(self.handEyeMat, hmInput)
                xyzuvw = HMUtil.convertHMtoXYZABCDeg(hmResult)

                # this conversion is moved to Operato, but come back by making robot move policy consistent
                if xyzuvw != None:
                    [x, y, z, u, v, w] = xyzuvw
                    # indy7 base position to gripper position
                    # xyzuvw = [x, y, z, u*(-1), v+180.0, w]
                    # NOTE: using the fixed rotation information
                    xyzuvw = [x, y, z, 180.0, 0.0, 180.0]

                # set final target position..
                markerObject.targetPos = xyzuvw

                # append this object to the list to be returned
                resultList.append(markerObject)
        else:
            pass

        return resultList

    def getMaskImage(self, color_image, width, height):
        assert self.maskdetect is not None
        mask_list = self.maskdetect.detect_object_by_data(color_image)

        return self.maskdetect.get_mask_image(mask_list, width, height)

    def putScoreData(self, color_image):
        assert color_image is not None

        # prepare font data
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        fontColor = (255, 255, 0)
        lineType = 1

        for idx, (cpoint, score) in enumerate(zip(self.center_point_list, self.scores_list)):
            print(idx, (cpoint, score))
            cv2.putText(color_image, '%0.8f' % score, cpoint, font,
                        fontScale, fontColor, lineType)

    def getScoresList(self):
        return self.scores_list
