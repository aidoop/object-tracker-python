
import numpy as np
import cv2
import sys, os
import cv2.aruco as aruco

import time

# add src root directory to python path
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )

import Config
from packages.CameraDevRealsense import RealsenseCapture
from packages.CameraVideoCapture import VideoCapture
from ObjectArucoMarkerTracker import ArucoMarkerObject, ArucoMarkerTracker
from ObjectTrackingKeyHandler import ObjectTrackingKeyHandler
from ROIRetangleManager import ROIRetangleManager


###############################################################################
# Hand-eye calibration process 
#   -                                                                
###############################################################################

if __name__ == '__main__':

    # create the camera device object of intel realsense
    rsCamDev = RealsenseCapture(0)

    # create video capture object using realsense camera device object
    vcap = VideoCapture(rsCamDev, 1280, 720, 30)

    # Start streaming
    vcap.start()

    # get instrinsics
    mtx, dist = vcap.getIntrinsicsMat(Config.UseRealSenseInternalMatrix)

    # create objs and an object tracker 
    # TODO: abstarcatoin.............
    objTracker = ArucoMarkerTracker()
    objTracker.initialize(aruco.DICT_5X5_250, 0.05)
    
    obj = ArucoMarkerObject(14, [[0.0, 0.0, 0.10, 0, 0, 0]])    
    objTracker.setTrackingObject(obj)
    obj = ArucoMarkerObject(30, [[0.0, 0.0, 0.10, 0, 0, 0]])    
    objTracker.setTrackingObject(obj)

    # load ROI Region from a json file
    ROIMgr = ROIRetangleManager()
    
    ROIRegionFile = cv2.FileStorage("ROIRegions.json", cv2.FILE_STORAGE_READ)
    roiCntNode = ROIRegionFile.getNode("ROICnt")
    roiCnt = int(roiCntNode.real())
    for idx in range(roiCnt):
        #roiNode = ROIRegionFile.getNode("ROI" + str(idx))
        ROIRegionNode = ROIRegionFile.getNode("ROI" + str(idx))
        ROIMgr.appendROI(ROIRegionNode.mat())
    ROIRegionFile.release()

    # creat key handler
    keyhandler = ObjectTrackingKeyHandler()

    # TODO: get duration (unit: sec)
    processDuration = 0.5

    #print("press 'c' to capture an image or press 'q' to exit...")
    try:
        while(True):
            # wait for a coherent pair of frames: depth and color
            color_image = vcap.getFrame()

            # detect markers here..
            resultObjs = objTracker.findObjects(color_image, mtx, dist)

            # draw ROI Region..
            for ROIRegion in ROIMgr.getROIList():
                cv2.rectangle(color_image, tuple(ROIRegion[0]), tuple(ROIRegion[1]), (255,0,0), 3)

            # iterate the final object list
            for resultObj in resultObjs:
                print(resultObj)
                print(ROIMgr.isInsideROI(resultObj.corners))
                # TODO: if any marker is inside ROI, send marker data to Node.JS using the result list here...

            # display the captured image
            cv2.imshow('Object Tracking Engine',color_image)

            # delay for the specific duration 
            time.sleep(processDuration)

            # handle key inputs
            pressedKey = (cv2.waitKey(1) & 0xFF)
            if keyhandler.processKeyHandler(pressedKey):
                break
    finally:
        # Stop streaming
        vcap.stop()

    cv2.destroyAllWindows()

