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
from packages.Util import ArucoTrackerErrMsg, ObjectTypeCheck
from ObjectUpdateStatus import ObjectUpdateStatus  
from ObjectArucoMarkerTracker import ArucoMarkerObject, ArucoMarkerTracker
from ObjectTrackingKeyHandler import ObjectTrackingKeyHandler
from ROIRetangleManager import ROIRetangleManager
from packages.VisionGqlClient import VisonGqlDataClient

class VisionTrackingCamera:
    name = None
    robotName = None        # TODO: how does camera make relation with robot???
    vcap = None
    mtx = None
    dist = None
    ROIMgr = None
    arucoMarkTracker = None
    handeye = None

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
    if Config.ObjTrackingDebugMode == False:
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
        vtc = VisionTrackingCamera()

        # set camera name
        vtc.name = trackingCamera.name

        # set robot name
        if ObjectTypeCheck.checkValueIsAvail(trackingCamera.baseRobotArm) == False:
            continue
        vtc.robotName = trackingCamera.baseRobotArm['name']

        # set camera endpoint
        #endpoint = int(trackingCamera.endpoint)
        
        # TODO: choose camera dev object based on camera type
        if trackingCamera.type == 'realsense-camera':
            rsCamDev = RealsenseCapture(trackingCamera.endpoint)
        elif trackingCamera.type == 'camera-connector':
            rsCamDev = OpencvCapture(int(trackingCamera.endpoint))

        # create a video capture object and start 
        vcap = VideoCapture(rsCamDev, Config.VideoFrameWidth, Config.VideoFrameHeight, Config.VideoFramePerSec, vtc.name)
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
        objTracker = ArucoMarkerTracker()
        objTracker.initialize(Config.ArucoDict, Config.ArucoSize, mtx, dist, vtc.handeye)
        vtc.arucoMarkTracker = objTracker

        # initialize ROI manager
        ROIMgr = ROIRetangleManager()
        for ROIData in trackingCamera.ROIs:
            ROIInput = [ROIData.tl[0], ROIData.tl[1], ROIData.rb[0], ROIData.rb[1], ROIData.id]
            ROIMgr.appendROI(ROIInput)
        vtc.ROIMgr = ROIMgr

        # setup an opencv window 
        # cv2.namedWindow(vtc.name, cv2.WINDOW_NORMAL)
        # cv2.setWindowProperty(vtc.name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        vtcList.append(vtc)
        idx+=1

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

        # marks doesn't have any dependency with camera, so all marks should be registered for all cameras
        obj = ArucoMarkerObject(int(trackableMark.endpoint), trackableMark.poseOffset)
        for vtc in vtcList:
            vtc.arucoMarkTracker.setTrackingObject(obj)

    # create key handler
    keyhandler = ObjectTrackingKeyHandler()

    # prepare object update status
    objStatusUpdate  = ObjectUpdateStatus(gqlDataClient.client)

    try:
        while(True):

            # get markable object 
            tobjIDList = vtc.arucoMarkTracker.getTrackingObjIDList().copy()

            for vtc in vtcList:
                # get a frame 
                color_image = vtc.vcap.getFrame()

                # check if core variables are available..
                if ArucoTrackerErrMsg.checkValueIsNone(vtc.mtx, "camera matrix") == False:
                    break
                if ArucoTrackerErrMsg.checkValueIsNone(vtc.dist, "distortion coeff.") == False:
                    break
                if ArucoTrackerErrMsg.checkValueIsNone(color_image, "video color frame") == False:
                    break
                if ArucoTrackerErrMsg.checkValueIsNone(vtc.handeye, "hand eye matrix") == False:
                    break

                # find a robot arm related with the current camera.
                for ra in raList:
                    if ra.name == vtc.robotName:
                        # detect markers here..
                        resultObjs = vtc.arucoMarkTracker.findObjects(color_image, vtc, ra.gripperOffset)

                        for resultObj in resultObjs:
                            (found, foundRIDs) = vtc.ROIMgr.isInsideROI(resultObj.corners)
                            [x, y, z, u, v, w] = resultObj.targetPos

                            

                            if found is True:
                                objStatusUpdate.addObjStatus(resultObj.markerID, foundRIDs, x, y, z, u, v, w)
                            else:
                                objStatusUpdate.addObjStatus(resultObj.markerID, [None], x, y, z, u, v, w)
                            tobjIDList.remove(resultObj.markerID)

                # draw ROI Region..
                for ROIRegion in vtc.ROIMgr.getROIList():
                    cv2.rectangle(color_image, (ROIRegion[0], ROIRegion[1]), (ROIRegion[2], ROIRegion[3]), (255,0,0), 3)

                # display a video by half of original size
                #color_image_half = cv2.resize(color_image, (960, 540)) 
                cv2.imshow(vtc.name, color_image)

            # send object information to UI and clear all
            #if objStatusUpdate.containsObjStatus() == True:
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
        # Stop streaming
        for vtc in vtcList:
            vtc.vcap.stop()


    cv2.destroyAllWindows()


