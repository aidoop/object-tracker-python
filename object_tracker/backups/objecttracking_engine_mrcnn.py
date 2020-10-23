import cv2
import cv2.aruco as aruco

import sys
import os
import argparse
import time
import numpy as np

from aidobjtrack.config.appconfig import AppConfig
from aidobjtrack.camera.camera_dev_realsense import RealsenseCapture
from aidobjtrack.camera.camera_dev_opencv import OpencvCapture
from aidobjtrack.camera.camera_videocapture import VideoCapture
from aidobjtrack.util.util import ObjectTrackerErrMsg, ObjectTypeCheck
from aidobjtrack.data_update.objecttracking_updatestatus import ObjectUpdateStatus
from aidobjtrack.keyhandler.objecttracking_keyhandler import ObjectTrackingKeyHandler
from aidobjtrack.visiongql.visiongql_client import VisonGqlDataClient

from aidobjtrack.objtracking.objecttracking_aurco import ArucoMarkerObject, ArucoMarkerTracker
from aidobjtrack.objtracking.objecttracking_mrcnn import MrcnnObject, MrcnnObjectTracker
from aidobjtrack.objtracking.objecttracking_roimgr_retangle import ROIRetangleManager

# mask rcnn detector
from mrcnn import inference as mo


class VisionTrackingCamera:
    name = None
    robotName = None
    vcap = None
    mtx = None
    dist = None
    ROIMgr = None
    objectMarkTracker = None
    handeye = None
    camObjOffset = None
    # maskdetect = None


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
    camKeys = gqlDataClient.trackingCameras.keys()
    for camKey in camKeys:
        # get gql camera information
        trackingCamera = gqlDataClient.trackingCameras[camKey]

        # create camera object
        vtc = VisionTrackingCamera()

        # set camera name
        vtc.name = trackingCamera.name

        # set robot name
        if ObjectTypeCheck.checkValueIsAvail(trackingCamera.baseRobotArm) == False:
            print("robot arm is not detected..")
            continue
        vtc.robotName = trackingCamera.baseRobotArm['name']

        # TODO: choose camera dev object based on camera type
        if trackingCamera.type == 'realsense-camera':
            rsCamDev = RealsenseCapture(trackingCamera.endpoint)
        elif trackingCamera.type == 'camera-connector':
            print("can't use camera-connector on mrcnn detector")
            continue

        # set to apply post-filters
        rsCamDev.set_flag_filters(True)
        rsCamDev.prepare_filters()

        # create a video capture object and start
        # TODO: separate configuration here for mask rcnn
        vcap = VideoCapture(rsCamDev, 848,
                            480, 30, vtc.name)
        if vcap == None:
            continue
        vcap.start()
        vtc.vcap = vcap

        # vtc.maskdetect = mo.MaskRcnnDetect(
        #     "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201006T1521/mask_rcnn_object-train_0047.h5", "/home/jinwon/Documents/github/object-tracker-python/logs")

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
        objTracker.initialize(vtc.handeye)
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

        if trackableMark.endpoint == '100':
            # marks doesn't have any dependency with camera, so all marks should be registered for all cameras
            obj = MrcnnObject(
                trackableMark.endpoint, trackableMark.poseOffset)
            for vtc in vtcList:
                vtc.objectMarkTracker.setTrackingObject(obj)
        else:
            # marks doesn't have any dependency with camera, so all marks should be registered for all cameras
            obj = ArucoMarkerObject(
                int(trackableMark.endpoint), trackableMark.poseOffset)
            for vtc in vtcList:
                vtc.objectMarkTracker.setTrackingObject(obj)

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
                (color_image, depth_image) = vcap.getFrame()

                ############################################################
                # detect objects

                # if vtc.maskdetect is not None:
                #     mask_list = vtc.maskdetect.detect_object_by_data(
                #         color_image)

                #     center_point_list = vtc.maskdetect.get_center_points(
                #         mask_list)
                #     print('center point: ', center_point_list)

                #     mask_image = vtc.maskdetect.get_mask_image(
                #         mask_list, 848, 480)
                #     cv2.imshow('mask', mask_image)

                #     # TODO: create ObjectTracker subclass here for maskrcnn detector
                #     # ....

                # find a robot arm related with the current camera.
                for ra in raList:
                    if ra.name == vtc.robotName:
                        # detect markers here..
                        resultObjs = vtc.objectMarkTracker.findObjects(
                            color_image, vtc, ra.gripperOffset)

                        if(resultObjs == None):
                            continue

                        # check if rect ROI is available for the current detection
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

                # # draw ROI Region..
                # for ROIRegion in vtc.ROIMgr.getROIList():
                #     cv2.rectangle(
                #         color_image, (ROIRegion[0], ROIRegion[1]), (ROIRegion[2], ROIRegion[3]), (255, 0, 0), 3)

                # BGR to RGB for opencv imshow function
                color_image_view = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                    depth_image, alpha=0.03), cv2.COLORMAP_JET)
                images = np.hstack((color_image_view, depth_colormap))

                # display a video by half of original size
                cv2.imshow(vtc.name, images)

            # send object information to UI and clear all
            # if objStatusUpdate.containsObjStatus() == True:
            # TODO: find object status update
            objStatusUpdate.sendObjStatus(tobjIDList)
            objStatusUpdate.clearObjStatus()

            # sleep for the specified duration.
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