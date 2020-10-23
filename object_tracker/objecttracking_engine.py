import cv2
import cv2.aruco as aruco

import sys
import os
import argparse
import time
import numpy as np

from aidobjtrack.config.appconfig import AppConfig, ObjectTrackingMethod
from aidobjtrack.objtracking.objecttracking_app_data import ObjectTrakcingAppData
from aidobjtrack.keyhandler.objecttracking_keyhandler import ObjectTrackingKeyHandler
from aidobjtrack.data_update.objecttracking_updatestatus import ObjectUpdateStatus
from aidobjtrack.util.util import ObjectTrackerErrMsg

###############################################################################
# Hand-eye calibration process
#   -
###############################################################################
if __name__ == '__main__':

    # create application data
    app_data = ObjectTrakcingAppData()
    if app_data.connect_server('http://localhost:3000', 'system', 'admin@hatiolab.com', 'admin') == False:
        print("Can't connect operato vision server.")
        sys.exit()

    app_data.parse()

    #########################################################################
    # create key handler
    keyhandler = ObjectTrackingKeyHandler()

    #########################################################################
    # prepare object update status
    objStatusUpdate = ObjectUpdateStatus(app_data.get_gql_client())

    try:
        while(True):

            # get markable object
            tobjIDList = app_data.get_mark_id_list_of_camera(0).copy()

            vtcList = app_data.get_camera_list()
            for vtc in vtcList:
                # get a frame
                color_image = vtc.vcap.getFrame()
                assert color_image is not None, 'no captured frame'

                # check if core variables are available..
                if (app_data.tracking_method == ObjectTrackingMethod.ARUCO):
                    if (ObjectTrackerErrMsg.checkValueIsNone(vtc.mtx, "camera matrix") == False) or (ObjectTrackerErrMsg.checkValueIsNone(vtc.dist, "distortion coeff.") == False):
                        break
                if ObjectTrackerErrMsg.checkValueIsNone(color_image, "video color frame") == False:
                    break
                if ObjectTrackerErrMsg.checkValueIsNone(vtc.handeye, "hand eye matrix") == False:
                    break

                # find a robot arm related with the current camera.
                raList = app_data.robot_arm_list
                for ra in raList:
                    if ra.name == vtc.robot_name:
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

                # display a frame image
                if app_data.tracking_method == ObjectTrackingMethod.ARUCO:
                    color_image_half = cv2.resize(
                        color_image, (int(AppConfig.VideoFrameWidth/2), int(AppConfig.VideoFrameHeight/2))) if AppConfig.VideoFrameWidth > 100 else color_image
                    cv2.imshow(vtc.camera_name, color_image_half)
                else:
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
                    cv2.imshow(vtc.camera_name, images)

            # send object information to UI and clear all
            # if objStatusUpdate.containsObjStatus() == True:
            objStatusUpdate.sendObjStatus(tobjIDList)
            objStatusUpdate.clearObjStatus()

            # sleep for the specified duration.
            if app_data.tracking_method == ObjectTrackingMethod.ARUCO:
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
