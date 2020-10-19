import cv2
import cv2.aruco as aruco

import sys
import os
import argparse
import time
import numpy as np
from enum import Enum, unique

from aidobjtrack.config.appconfig import AppConfig
from aidobjtrack.camera.camera_dev_realsense import RealsenseCapture
from aidobjtrack.camera.camera_dev_opencv import OpencvCapture
from aidobjtrack.camera.camera_videocapture import VideoCapture
from aidobjtrack.util.util import ObjectTrackerErrMsg, ObjectTypeCheck
from aidobjtrack.data_update.objecttracking_updatestatus import ObjectUpdateStatus
from aidobjtrack.keyhandler.objecttracking_keyhandler import ObjectTrackingKeyHandler
from aidobjtrack.visiongql.visiongql_client import VisonGqlDataClient

from objecttracking_aurco import ArucoMarkerObject, ArucoMarkerTracker
from objecttracking_mrcnn import MrcnnObject, MrcnnObjectTracker
from objecttracking_roimgr_retangle import ROIRetangleManager

# mask rcnn detector
from mrcnn import object_detect as mo


@unique
class ObjectTrackingMethod(Enum):
    ARUCO = 1
    MRCNN = 2


class ObjectTrakcingData:
    # set trakcing method
    tracking_method = ObjectTrackingMethod.MRCNN
    name = None
    robotName = None
    vcap = None
    mtx = None
    dist = None
    ROIMgr = None
    objectMarkTracker = None
    handeye = None
    camObjOffset = None


###############################################################################
# Hand-eye calibration process
#   -
###############################################################################
if __name__ == '__main__':

    # TODO: get object, camera workspace from gql server
    gqlDataClient = VisonGqlDataClient()
    if(gqlDataClient.connect('http://localhost:3000', 'system', 'admin@hatiolab.com', 'admin') is False):
        print("Can't connect operato vision server.")
        sys.exit()

    # get gql data for a workspace
    if AppConfig.ObjTrackingDebugMode == False:
        gqlDataClient.fetchVisionWorkspace()
    else:
        gqlDataClient.fetchTrackingCamerasAll()
        gqlDataClient.fetchRobotArmsAll()
        gqlDataClient.fetchTrackableMarksAll()

    #####################################
    # application data list
    # vistion tracking camera object list
    vtcList = list()
    # vision robot arm list
    raList = list()

    #########################################################################
    # intitialize tracking cameras
    idx = 0
    camKeys = gqlDataClient.trackingCameras.keys()
    for camKey in camKeys:
        # get gql camera information
        trackingCamera = gqlDataClient.trackingCameras[camKey]

        # create camera object
        vtc = ObjectTrakcingData()

        # set camera name
        vtc.name = trackingCamera.name

        # set robot name
        if ObjectTypeCheck.checkValueIsAvail(trackingCamera.baseRobotArm) == False:
            print("robot arm is not detected..")
            continue
        vtc.robotName = trackingCamera.baseRobotArm['name']

        # set camera endpoint
        #endpoint = int(trackingCamera.endpoint)

        # TODO: choose camera dev object based on camera type
        if trackingCamera.type == 'realsense-camera':
            rsCamDev = RealsenseCapture(trackingCamera.endpoint)
            # set to apply post-filters
            rsCamDev.set_flag_filters(True)
            rsCamDev.prepare_filters()
        elif trackingCamera.type == 'camera-connector':
            rsCamDev = OpencvCapture(int(
                trackingCamera.endpoint)) if vtc.tracking_method == ObjectTrackingMethod.ARUCO else None

        if rsCamDev is None:
            print('camera type error')
            continue

        # create a video capture object and start
        vcap = VideoCapture(rsCamDev, AppConfig.VideoFrameWidth,
                            AppConfig.VideoFrameHeight, AppConfig.VideoFramePerSec, vtc.name)
        if vcap == None:
            continue
        vcap.start()
        vtc.vcap = vcap

        # set camera matrix and distortion coefficients
        mtx = trackingCamera.cameraMatrix
        dist = trackingCamera.distCoeff
        vtc.mtx = mtx
        vtc.dist = dist

        # set hand eye matrix
        vtc.handeye = trackingCamera.handEyeMatrix

        # create aurco mark tracker object
        objTracker = MrcnnObjectTracker()
        # TODO: change fucntion parameters for the comparability of ArucoObjectTracker
        objTracker.initialize(
            AppConfig.ArucoDict, AppConfig.ArucoSize, mtx, dist, vtc.handeye)
        vtc.objectMarkTracker = objTracker

        # initialize ROI manager
        ROIMgr = ROIRetangleManager()
        for ROIData in trackingCamera.ROIs:
            ROIInput = [ROIData.tl[0], ROIData.tl[1],
                        ROIData.rb[0], ROIData.rb[1], ROIData.id]
            ROIMgr.appendROI(ROIInput)
        vtc.ROIMgr = ROIMgr

        vtc.camObjOffset = trackingCamera.camObjOffset

        # setup an opencv window
        # cv2.namedWindow(vtc.name, cv2.WINDOW_NORMAL)
        # cv2.setWindowProperty(vtc.name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        vtcList.append(vtc)
        idx += 1

    #########################################################################
    # initialize robot arms
    robotArmKeys = gqlDataClient.robotArms.keys()
    for robotArmKey in robotArmKeys:
        robotArm = gqlDataClient.robotArms[robotArmKey]
        raList.append(robotArm)

    #########################################################################
    # intialize trackable marks.
    trackableMarkKeys = gqlDataClient.trackableObjects.keys()
    for trackableMarkKey in trackableMarkKeys:
        trackableMark = gqlDataClient.trackableObjects[trackableMarkKey]
        print('Trackable Marks')
        print(trackableMark.endpoint, ', ', trackableMark.poseOffset)

        if vtc.tracking_method == ObjectTrackingMethod.MRCNN:
            # marks doesn't have any dependency with camera, so all marks should be registered for all cameras
            obj = MrcnnObject(
                trackableMark.endpoint, trackableMark.poseOffset)
            for vtc in vtcList:
                vtc.objectMarkTracker.setTrackingObject(obj)
        elif vtc.tracking_method == ObjectTrackingMethod.ARUCO:
            # marks doesn't have any dependency with camera, so all marks should be registered for all cameras
            obj = ArucoMarkerObject(
                int(trackableMark.endpoint), trackableMark.poseOffset)
            for vtc in vtcList:
                vtc.objectMarkTracker.setTrackingObject(obj)
        else:
            pass

    # create key handler
    keyhandler = ObjectTrackingKeyHandler()

    # prepare object update status
    objStatusUpdate = ObjectUpdateStatus(gqlDataClient.client)

    try:
        while(True):

            # get markable object
            tobjIDList = vtc.objectMarkTracker.getTrackingObjIDList().copy()

            for vtc in vtcList:
                # get a frame
                if vtc.tracking_method == ObjectTrackingMethod.ARUCO:
                    color_image = vtc.vcap.getFrame()
                elif vtc.tracking_method == ObjectTrackingMethod.MRCNN:
                    (color_image, depth_image) = vcap.getFrame()

                # check if core variables are available..
                if (vtc.tracking_method == ObjectTrackingMethod.ARUCO):
                    if (ObjectTrackerErrMsg.checkValueIsNone(vtc.mtx, "camera matrix") == False) or (ObjectTrackerErrMsg.checkValueIsNone(vtc.dist, "distortion coeff.") == False):
                        break
                if ObjectTrackerErrMsg.checkValueIsNone(color_image, "video color frame") == False:
                    break
                if ObjectTrackerErrMsg.checkValueIsNone(vtc.handeye, "hand eye matrix") == False:
                    break

                # find a robot arm related with the current camera.
                for ra in raList:
                    if ra.name == vtc.robotName:
                        # detect markers here..
                        resultObjs = vtc.objectMarkTracker.findObjects(
                            color_image, vtc, ra.gripperOffset)

                        if(resultObjs == None):
                            continue

                        for resultObj in resultObjs:
                            # # TODO: check if rect ROI is available for the current detection
                            # (found, foundRIDs) = vtc.ROIMgr.isInsideROI(
                            #     resultObj.corners)
                            found = False

                            if resultObj.targetPos is not None:
                                [x, y, z, u, v, w] = resultObj.targetPos

                                if found is True:
                                    objStatusUpdate.addObjStatus(
                                        resultObj.markerID, foundRIDs, x, y, z, u, v, w)
                                else:
                                    objStatusUpdate.addObjStatus(
                                        resultObj.markerID, [None], x, y, z, u, v, w)
                                if len(tobjIDList) > 0:
                                    tobjIDList.remove(resultObj.markerID)

                # draw ROI Region..
                # for ROIRegion in vtc.ROIMgr.getROIList():
                #     cv2.rectangle(
                #         color_image, (ROIRegion[0], ROIRegion[1]), (ROIRegion[2], ROIRegion[3]), (255, 0, 0), 3)

                # display a video by half of original size
                if vtc.tracking_method == ObjectTrackingMethod.ARUCO:
                    color_image_half = cv2.resize(color_image, (960, 540))
                    cv2.imshow(vtc.name, color_image_half)
                else:
                    # TODO: need to show the depth image here?
                    # BGR to RGB for opencv imshow function
                    color_image_view = cv2.cvtColor(
                        color_image, cv2.COLOR_RGB2BGR)

                    # create images to show
                    # sub_image = cv2.applyColorMap(cv2.convertScaleAbs(
                    #     depth_image, alpha=0.03), cv2.COLORMAP_JET)
                    mask_image = vtc.objectMarkTracker.getMaskImage(
                        color_image, AppConfig.VideoFrameWidth, AppConfig.VideoFrameHeight)
                    sub_image = cv2.cvtColor(mask_image, cv2.COLOR_GRAY2RGB)

                    # show scores
                    vtc.objectMarkTracker.putScoreData(color_image_view)

                    # display both color image and mask image
                    images = np.hstack((color_image_view, sub_image))
                    cv2.imshow(vtc.name, images)

            # send object information to UI and clear all
            # if objStatusUpdate.containsObjStatus() == True:
            objStatusUpdate.sendObjStatus(tobjIDList)
            objStatusUpdate.clearObjStatus()

            # sleep for the specified duration.
            if vtc.tracking_method == ObjectTrackingMethod.ARUCO:
                time.sleep(0.2)

            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhandler.processKeyHandler(pressedKey):
                break

    except Exception as ex:
        print("Error :", ex)

    finally:
        for vtc in vtcList:
            vtc.vcap.stop()

    cv2.destroyAllWindows()
