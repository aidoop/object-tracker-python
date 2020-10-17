import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import os

from aidobjtrack.abc.objecttracking_base import ObjectTracker
from aidobjtrack.handeye.calibhandeye_handeye import HandEyeCalibration
from aidobjtrack.util.hm_util import HMUtil

# mask rcnn detector
from mrcnn import object_detect as mo


class MrcnnObject:
    def __init__(self, markerID, pivotOffset):
        self.markerID = markerID
        self.pivotOffset = pivotOffset
        self.targetPos = None


class MrcnnObjectTracker(ObjectTracker):
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201006T1521/mask_rcnn_object-train_0047.h5"
    #OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201016T2305/mask_rcnn_object-train_0035.h5"
    OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201017T1422/mask_rcnn_object-train_0043.h5"
    LOGS_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs"

    def __init__(self):
        self.markerObjectList = []
        self.markerObjIDList = []

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

            center_point_list = self.maskdetect.get_center_points(
                mask_list)
            print('center point: ', center_point_list)

            # show the mask image
            mask_image = self.maskdetect.get_mask_image(
                mask_list, 848, 480)
            cv2.imshow('mask', mask_image)

            for markerObject in self.markerObjectList:
                for cpoint in center_point_list:
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

                    if vtc.camObjOffset is not None:
                        offsetPoint = markerObject.pivotOffset + vtc.camObjOffset
                    else:
                        offsetPoint = markerObject.pivotOffset

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
                        #xyzuvw = [x, y, z, u*(-1), v+180.0, w]
                        xyzuvw = [x, y, z, 180.0, 0.0, 180.0]

                    # set final target position..
                    markerObject.targetPos = xyzuvw

                    # append this object to the list to be returned
                    resultList.append(markerObject)
        else:
            pass

        return resultList
