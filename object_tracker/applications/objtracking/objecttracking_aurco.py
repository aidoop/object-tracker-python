import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import os

from aidoop.calibration.calibhandeye_handeye import HandEyeCalibration
from aidoop.etc.hm_util import HMUtil
from aidoop.aruco.aruco_detect import ArucoDetect
from aidoop.aruco.aruco_advanced_pose import ArucoAdvPose

from applications.objtracking.objecttracking_base import ObjectTracker


class ArucoMarkerObject:
    def __init__(self, markerID, pivotOffset):
        self.markerID = markerID
        self.pivotOffset = pivotOffset
        self.corners = None
        self.targetPos = None


class ArucoMarkerTracker(ObjectTracker):
    def __init__(self):
        self.markerObjectList = []
        self.markerObjIDList = []

    # initialize parameters for any camera operation
    def initialize(self, *args):
        self.markerSelectDict = args[0]
        self.markerSize = args[1]
        self.camMtx = args[2]
        self.dist = args[3]
        self.markerObjectList.clear()
        self.markerObjIDList.clear()
        self.handEyeMat = args[4]  # HandEyeCalibration.loadTransformMatrix()
        self.arucoDetect = ArucoDetect(
            self.markerSelectDict, self.markerSize, self.camMtx, self.dist
        )
        self.arucoAdvPose = ArucoAdvPose()

    # set detectable features like marker id of aruco marker
    def set_tracking_object(self, object):
        assert isinstance(object, ArucoMarkerObject)

        found = False
        for mo in self.markerObjectList:
            if object.markerID == mo.markerID:
                found = True

        if found is False:
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
        mtx = vtc.mtx
        dist = vtc.dist
        gripperOffset = args[2]

        # prepare list to give over the result objects
        resultList = list()

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # # lists of ids and the corners belonging to each id
        corners, ids = self.arucoDetect.detect(gray)

        # set detected objects to the result list..
        if np.all(ids != None):
            # estimate pose of each marker and return the values
            rvec, tvec = self.arucoDetect.estimate_pose(corners)

            for markerObject in self.markerObjectList:
                for idx in range(len(ids)):
                    if ids[idx] == markerObject.markerID:
                        # set additional properties to this found object..
                        # print(vtc.name, ") Found Target ID: " + str(markerObject.markerID))
                        markerObject.corners = corners[idx].reshape(
                            4, 2
                        )  # reshape: (1,4,2) --> (4,2)

                        # change a rotation vector to a rotation matrix
                        rotMatrix = np.zeros(shape=(3, 3))
                        cv2.Rodrigues(rvec[idx], rotMatrix)

                        # make a homogeneous matrix using a rotation matrix and a translation matrix
                        hmCal2Cam = HMUtil.create_homogeneous_matrix(
                            rotMatrix, tvec[idx]
                        )
                        # xyzuvw_midterm = HMUtil.convert_hm_to_xyzabc_by_deg(hmCal2Cam)
                        # print("Esitmated Mark Pose: ", xyzuvw_midterm)

                        # calcaluate the modified position based on pivot offset
                        # if markerObject.pivotOffset is None:
                        #     hmWanted = HMUtil.convert_xyzabc_to_hm_by_deg([0.0, 0.0, 0.00, 0.0, 0.0, 0.0])     # fix z + 0.01 regardless of some input offsets like tool offset, poi offset,...
                        #     hmInput = np.dot(hmCal2Cam, hmWanted)
                        # else:
                        #     hmWanted = HMUtil.convert_xyzabc_to_hm_by_deg(markerObject.pivotOffset)
                        #     hmInput = np.dot(hmCal2Cam, hmWanted)

                        # update a pivot offset
                        vtc.camObjOffset = (
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                            if vtc.camObjOffset is None
                            else vtc.camObjOffset
                        )
                        markerObject.pivotOffset = (
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                            if markerObject.pivotOffset is None
                            else markerObject.pivotOffset
                        )
                        if vtc.camObjOffset is not None:
                            offsetPoint = markerObject.pivotOffset + vtc.camObjOffset
                        else:
                            offsetPoint = markerObject.pivotOffset

                        # fix z + 0.01 regardless of some input offsets like tool offset, poi offset,...
                        hmWanted = HMUtil.convert_xyzabc_to_hm_by_deg(offsetPoint)
                        hmInput = np.dot(hmCal2Cam, hmWanted)

                        ###########################################################################################
                        # TODO: should find out how to apply tool offset and poi offset here...

                        # get a final position
                        hmResult = np.dot(self.handEyeMat, hmInput)
                        xyzuvw = HMUtil.convert_hm_to_xyzabc_by_deg(hmResult)

                        # applying advance pose model
                        [x, y, z, u, v, w] = xyzuvw
                        self.arucoAdvPose.set_pose(x, y, z, u, v, w)

                        if self.arucoAdvPose.stable() == True:
                            xyzuvw = self.arucoAdvPose.get_filtered_poses()

                        # TODO:.............................................
                        # TODO: should change this routine....

                        # this conversion is moved to Operato, but come back by making robot move policy consistent
                        if xyzuvw != None:
                            [x, y, z, u, v, w] = xyzuvw
                            # indy7 base position to gripper position
                            xyzuvw = [x, y, z, u * (-1), v + 180.0, w]

                        # set final target position..
                        markerObject.targetPos = xyzuvw

                        # print("Final XYZUVW: ", xyzuvw)

                        # append this object to the list to be returned
                        resultList.append(markerObject)

                        # draw a cooordinate axis(x, y, z)
                        aruco.drawAxis(
                            color_image, mtx, dist, rvec[idx], tvec[idx], 0.02
                        )

            # draw a square around the markers
            aruco.drawDetectedMarkers(color_image, corners)

        return resultList
