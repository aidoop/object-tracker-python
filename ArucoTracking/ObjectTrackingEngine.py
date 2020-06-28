import cv2
import cv2.aruco as aruco

import sys, os
import argparse
import time

# add src root directory to python path
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )
import Config
from packages.CameraDevRealsense import RealsenseCapture
from packages.CameraDevOpencv import OpencvCapture
from packages.CameraVideoCapture import VideoCapture
from ObjectArucoMarkerTracker import ArucoMarkerObject, ArucoMarkerTracker
from ObjectTrackingKeyHandler import ObjectTrackingKeyHandler
from ROIRetangleManager import ROIRetangleManager
from VisionGqlClient import VisonGqlDataClient

class VisionTrackingCamera:
    name = None
    robotName = None        # TODO: how does camera make relation with robot???
    vcap = None
    mtx = None
    dist = None
    ROIMgr = None
    arucoMarkTracker = None

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

    #gqlDataClient.parseVisionWorkspaces()
    # process all elements here...
    gqlDataClient.fetchTrackingCameras()
    gqlDataClient.fetchRobotArms()
    gqlDataClient.fetchTrackableMarks()

    #####################################
    # application data list
    # vistion tracking camera object list
    vtcList = list()
    # vision robot arm list
    raList = list()

    #########################################################################
    # initialize robot arms
    robotArmKeys = gqlDataClient.robotArms.keys()
    for robotArmKey in robotArmKeys:
        robotArm = gqlDataClient.robotArms[robotArmKey]
        print(robotArm.gripperOffset)
        raList.append(robotArm)

    #########################################################################
    # intitialize tracking cameras
    idx = 0
    camKeys = gqlDataClient.trackingCameras.keys()
    for camKey in camKeys:
        # get gql camera information 
        trackingCamera = gqlDataClient.trackingCameras[camKey]

        # create camera object 
        vtc = VisionTrackingCamera()

        # set camera name
        vtc.name = trackingCamera.name

        # set camera endpoint
        endpoint = int(trackingCamera.endpoint)
        
        # TODO: choose camera dev object based on camera type
        if trackingCamera.type == 'realsense-camera':
            rsCamDev = RealsenseCapture(endpoint)
        elif trackingCamera.type == 'camera-connector':
            rsCamDev = OpencvCapture(endpoint)

        # create a video capture object and start 
        vcap = VideoCapture(rsCamDev, Config.VideoFrameWidth, Config.VideoFrameHeight, Config.VideoFramePerSec)
        if vcap == None:
            continue
        vcap.start()
        vtc.vcap = vcap

        # set camera matrix and distortion coefficients
        mtx = trackingCamera.cameraMatrix
        dist = trackingCamera.distCoeff
        vtc.mtx = mtx
        vtc.dist = dist

        # create aurco mark tracker object
        objTracker = ArucoMarkerTracker()
        objTracker.initialize(Config.ArucoDict, Config.ArucoSize, mtx, dist)
        vtc.arucoMarkTracker = objTracker

        # initialize ROI manager
        ROIMgr = ROIRetangleManager()
        for ROIData in trackingCamera.ROIs:
            ROIInput = [ROIData.tl[0], ROIData.tl[1], ROIData.rb[0], ROIData.rb[1]]
        ROIMgr.appendROI(ROIInput)
        vtc.ROIMgr = ROIMgr

        vtcList.append(vtc)
        idx+=1

    #########################################################################
    # intialize trackable marks. 
    trackableMarkKeys = gqlDataClient.trackableObjects.keys()
    for trackableMarkKey in trackableMarkKeys:
        trackableMark = gqlDataClient.trackableObjects[trackableMarkKey]
        print('Trackable Marks')
        print(trackableMark.endpoint, ', ', trackableMark.poseOffset)

        # marks doesn't have any dependency with camera, so all marks should be registered for all cameras
        obj = ArucoMarkerObject(int(trackableMark.endpoint), trackableMark.poseOffset)
        for vtc in vtcList:
            vtc.arucoMarkTracker.setTrackingObject(obj)

    # create key handler
    keyhandler = ObjectTrackingKeyHandler()

    # prepare lists for mark tracking
    color_images = list()

    try:
        while(True):
            for vtc in vtcList:
                # get a frame 
                color_image = vtc.vcap.getFrame()

                # detect markers here..
                resultObjs = vtc.arucoMarkTracker.findObjects(color_image, vtc.mtx, vtc.dist)

                # check if an object is in ROI 
                for resultObj in resultObjs:
                    print(vtc.ROIMgr.isInsideROI(resultObj.corners))
                    # send object information to UI..

                # draw ROI Region..
                for ROIRegion in vtc.ROIMgr.getROIList():
                    cv2.rectangle(color_image, (ROIRegion[0], ROIRegion[1]), (ROIRegion[2], ROIRegion[3]), (255,0,0), 3)

                cv2.imshow(vtc.name, color_image)

            # sleep for the specific duration.
            time.sleep(0.1)            

            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhandler.processKeyHandler(pressedKey):
                break
    finally:
        # Stop streaming
        for vtc in vtcList:
            vtc.vcap.stop()

    cv2.destroyAllWindows()



