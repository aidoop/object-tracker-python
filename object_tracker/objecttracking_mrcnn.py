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
    OBJECT_WEIGHT_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201006T1521/mask_rcnn_object-train_0047.h5"
    LOGS_PATH = "/home/jinwon/Documents/github/object-tracker-python/logs"

    def __init__(self):
        self.markerObjectList = []

    # initialize parameters for any camera operation
    def initialize(self, *args):
        self.markerObjectList.clear()
        self.handEyeMat = args[4]  # HandEyeCalibration.loadTransformMatrix()

        self.maskdetect = mo.MaskRcnnDetect(
            MrcnnObjectTracker.OBJECT_WEIGHT_PATH, MrcnnObjectTracker.LOGS_PATH)

    # set detectable features like marker id of aruco marker
    def setTrackingObject(self, object):
        assert(isinstance(object, MrcnnObject))

        # TODO: handles two more object here?
        self.markerObjectList.append(object)

    def getTrackingObjectList(self):
        return self.markerObjectList

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

            # # show the mask image
            # mask_image = vtc.maskdetect.get_mask_image(
            #     mask_list, 848, 480)
            # cv2.imshow('mask', mask_image)

            for cpoint in center_point_list:
                tvec = vtc.vcap.get3DPosition(cpoint[0], cpoint[1])
                rvec = [-180, 0, -180]  # TODO: check it using real robot data

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
                hmWanted = HMUtil.convertXYZABCtoHMDeg(offsetPoint)
                hmInput = np.dot(hmCal2Cam, hmWanted)

                # get a final position
                hmResult = np.dot(self.handEyeMat, hmInput)
                xyzuvw = HMUtil.convertHMtoXYZABCDeg(hmResult)

                # this conversion is moved to Operato, but come back by making robot move policy consistent
                if xyzuvw != None:
                    [x, y, z, u, v, w] = xyzuvw
                    # indy7 base position to gripper position
                    xyzuvw = [x, y, z, u*(-1), v+180.0, w]

                # set final target position..
                markerObject.targetPos = xyzuvw

                # append this object to the list to be returned
                resultList.append(markerObject)
        else:
            pass

        # gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # # lists of ids and the corners belonging to each id
        # corners, ids = self.arucoDetect.detect(gray)

        # # set detected objects to the result list..
        # if np.all(ids != None):
        #     # estimate pose of each marker and return the values
        #     rvec, tvec = self.arucoDetect.estimatePose(corners)

        #     for markerObject in self.markerObjectList:
        #         for idx in range(len(ids)):
        #             if ids[idx] == markerObject.markerID:
        #                 # set additional properties to this found object..
        #                 #print(vtc.name, ") Found Target ID: " + str(markerObject.markerID))
        #                 markerObject.corners = corners[idx].reshape(
        #                     4, 2)    # reshape: (1,4,2) --> (4,2)

        #                 # change a rotation vector to a rotation matrix
        #                 rotMatrix = np.zeros(shape=(3, 3))
        #                 cv2.Rodrigues(rvec[idx], rotMatrix)

        #                 # make a homogeneous matrix using a rotation matrix and a translation matrix
        #                 hmCal2Cam = HMUtil.makeHM(rotMatrix, tvec[idx])
        #                 #xyzuvw_midterm = HMUtil.convertHMtoXYZABCDeg(hmCal2Cam)
        #                 #print("Esitmated Mark Pose: ", xyzuvw_midterm)

        #                 # calcaluate the modified position based on pivot offset
        #                 # if markerObject.pivotOffset is None:
        #                 #     hmWanted = HMUtil.convertXYZABCtoHMDeg([0.0, 0.0, 0.00, 0.0, 0.0, 0.0])     # fix z + 0.01 regardless of some input offsets like tool offset, poi offset,...
        #                 #     hmInput = np.dot(hmCal2Cam, hmWanted)
        #                 # else:
        #                 #     hmWanted = HMUtil.convertXYZABCtoHMDeg(markerObject.pivotOffset)
        #                 #     hmInput = np.dot(hmCal2Cam, hmWanted)

        #                 # update a pivot offset
        #                 if vtc.camObjOffset is not None:
        #                     offsetPoint = markerObject.pivotOffset + vtc.camObjOffset
        #                 else:
        #                     offsetPoint = markerObject.pivotOffset

        #                 # fix z + 0.01 regardless of some input offsets like tool offset, poi offset,...
        #                 hmWanted = HMUtil.convertXYZABCtoHMDeg(offsetPoint)
        #                 hmInput = np.dot(hmCal2Cam, hmWanted)

        #                 ###########################################################################################
        #                 # TODO: should find out how to apply tool offset and poi offset here...

        #                 # get a final position
        #                 hmResult = np.dot(self.handEyeMat, hmInput)
        #                 xyzuvw = HMUtil.convertHMtoXYZABCDeg(hmResult)

        #                 # applying advance pose model
        #                 [x, y, z, u, v, w] = xyzuvw
        #                 self.arucoAdvPose.setPose(x, y, z, u, v, w)

        #                 if(self.arucoAdvPose.stable() == True):
        #                     xyzuvw = self.arucoAdvPose.getPoses()

        #                 # this conversion is moved to Operato, but come back by making robot move policy consistent
        #                 if xyzuvw != None:
        #                     [x, y, z, u, v, w] = xyzuvw
        #                     # indy7 base position to gripper position
        #                     xyzuvw = [x, y, z, u*(-1), v+180.0, w]

        #                 # set final target position..
        #                 markerObject.targetPos = xyzuvw

        #                 # append this object to the list to be returned
        #                 resultList.append(markerObject)

        #                 # draw a cooordinate axis(x, y, z)
        #                 aruco.drawAxis(color_image, mtx, dist,
        #                                rvec[idx], tvec[idx], 0.02)

        #     # draw a square around the markers
        #     aruco.drawDetectedMarkers(color_image, corners)

        return resultList
