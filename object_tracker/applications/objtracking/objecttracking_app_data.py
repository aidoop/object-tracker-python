import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import os
from enum import Enum, unique

from aidoop.camera.camera_videocapture import VideoCaptureFactory

from applications.config.appconfig import AppConfig, ObjectTrackingMethod
from applications.etc.util import ObjectTypeCheck
from applications.visiongql.visiongql_client import VisonGqlDataClient
from applications.objtracking.objecttracking_aurco import (
    ArucoMarkerObject,
    ArucoMarkerTracker,
)
from applications.objtracking.objecttracking_roimgr_retangle import ROIRetangleManager


class ObjectTrakcingAppData(object):
    def __init__(self):
        # set trakcing method-
        self.tracking_method = AppConfig.APP_TRACKING_METHOD
        self.gqlDataClient = VisonGqlDataClient()

        self.obj_tracking_camera_list = list()
        self.robot_arm_list = list()

    def connect_server(self, url, domain, id, pw):
        return self.gqlDataClient.connect(url, domain, id, pw)

    def parse(self):
        try:
            # get gql data for a workspace
            if AppConfig.ObjTrackingDebugMode == False:
                self.gqlDataClient.fetch_vision_workspace()
            else:
                self.gqlDataClient.fetch_tracking_camera_all()
                self.gqlDataClient.fetch_robot_arm_all()
                self.gqlDataClient.fetch_trackable_objects_all()

            # print('***********************')
            # print('self.gqlDataClient.detectionMethod: ',
            #       self.gqlDataClient.detectionMethod)
            self.tracking_method = self.get_detection_method(
                self.gqlDataClient.detectionMethod
            )
            # print('self.tracking_method: ',
            #       self.tracking_method)
            if self.tracking_method == ObjectTrackingMethod.BOX:
                from applications.objtracking.objecttracking_box import (
                    BoxObject,
                    BoxObjectTracker,
                )

            ###############################################################################
            # most of data is based on camera connector of operato-robotics
            camKeys = self.gqlDataClient.trackingCameras.keys()
            for camKey in camKeys:
                # get gql camera information
                trackingCamera = self.gqlDataClient.trackingCameras[camKey]

                # create an ObjectTrakcingCamera object
                obj_tracking_camera = self.ObjectTrakcingCamera()

                # set camera name
                obj_tracking_camera.camera_name = trackingCamera.name

                # set robot name
                if AppConfig.ObjTrackingDebugWoRobot == False:
                    if (
                        ObjectTypeCheck.check_value_available(trackingCamera.baseRobotArm)
                        == False
                    ):
                        print("robot arm is not detected..")
                        continue
                    obj_tracking_camera.robot_name = trackingCamera.baseRobotArm["name"]

                # create a camera device object
                # rsCamDev = (
                #     RealsenseCapture(trackingCamera.endpoint)
                #     if trackingCamera.type == "realsense-camera"
                #     else (
                #         OpencvCapture(int(trackingCamera.endpoint))
                #         if trackingCamera.type == "camera-connector"
                #         else None
                #     )
                # )
                # assert rsCamDev is not None, "camera type error"
                # rsCamDev.prepare()

                # create a video capture object and start
                AppConfig.VideoFrameWidth = (
                    trackingCamera.width
                    if self.tracking_method is ObjectTrackingMethod.ARUCO
                    else AppConfig.VideoFrameWidth
                )
                AppConfig.VideoFrameHeight = (
                    trackingCamera.height
                    if self.tracking_method is ObjectTrackingMethod.ARUCO
                    else AppConfig.VideoFrameHeight
                )
                vcap = VideoCaptureFactory.create_video_capture(
                    trackingCamera.type,
                    trackingCamera.endpoint,
                    AppConfig.VideoFrameWidth,
                    AppConfig.VideoFrameHeight,
                    AppConfig.VideoFramePerSec,
                    trackingCamera.name,
                )
                # vcap = VideoCapture(
                #     rsCamDev,
                #     AppConfig.VideoFrameWidth,
                #     AppConfig.VideoFrameHeight,
                #     AppConfig.VideoFramePerSec,
                #     obj_tracking_camera.camera_name,
                # )

                if vcap == None:
                    continue
                vcap.start()
                obj_tracking_camera.vcap = vcap

                # set camera matrix and distortion coefficients
                mtx = trackingCamera.cameraMatrix
                dist = trackingCamera.distCoeff
                obj_tracking_camera.mtx = mtx
                obj_tracking_camera.dist = dist

                # set handeye automode flag
                obj_tracking_camera.handeyeAutoMode = trackingCamera.handEyeAutoMode

                # set handeye matrix
                obj_tracking_camera.handeye = trackingCamera.handEyeMatrix

                # create an object tracker
                objTracker = (
                    BoxObjectTracker()
                    if self.tracking_method == ObjectTrackingMethod.BOX
                    else ArucoMarkerTracker()
                    if self.tracking_method == ObjectTrackingMethod.ARUCO
                    else None
                )
                assert objTracker is not None

                # TODO: change fucntion parameters for the comparability of ArucoObjectTracker
                objTracker.initialize(
                    AppConfig.ArucoDict,
                    AppConfig.ArucoSize,
                    mtx,
                    dist,
                    obj_tracking_camera.handeye,
                )
                obj_tracking_camera.objectMarkTracker = objTracker

                # initialize ROI manager -
                # TODO: deprecated soon......
                ROIMgr = ROIRetangleManager()
                for ROIData in trackingCamera.ROIs:
                    ROIInput = [
                        ROIData.tl[0],
                        ROIData.tl[1],
                        ROIData.rb[0],
                        ROIData.rb[1],
                        ROIData.id,
                    ]
                    ROIMgr.append_roi(ROIInput)
                obj_tracking_camera.ROIMgr = ROIMgr

                obj_tracking_camera.camObjOffset = trackingCamera.camObjOffset

                # add the current camera data to list
                self.obj_tracking_camera_list.append(obj_tracking_camera)

            #########################################################################
            # initialize robot arms
            robotArmKeys = self.gqlDataClient.robotArms.keys()
            for robotArmKey in robotArmKeys:
                robotArm = self.gqlDataClient.robotArms[robotArmKey]
                self.robot_arm_list.append(robotArm)

            #########################################################################
            # intialize trackable marks.
            trackableMarkKeys = self.gqlDataClient.trackableObjects.keys()
            for trackableMarkKey in trackableMarkKeys:
                trackableMark = self.gqlDataClient.trackableObjects[trackableMarkKey]
                print("Trackable Marks")
                print(trackableMark.endpoint, ", ", trackableMark.poseOffset)

                # marks doesn't have any dependency with camera, so all marks should be registered for all cameras
                obj = (
                    BoxObject(trackableMark.endpoint, trackableMark.poseOffset)
                    if self.tracking_method == ObjectTrackingMethod.BOX
                    else ArucoMarkerObject(
                        int(trackableMark.endpoint), trackableMark.poseOffset
                    )
                    if self.tracking_method == ObjectTrackingMethod.ARUCO
                    else None
                )
                assert obj is not None, "trackable object not defined"

                for otc in self.obj_tracking_camera_list:
                    otc.objectMarkTracker.set_tracking_object(obj)

        except Exception as ex:
            print("application data parsing error :", ex)

    def get_mark_id_list_of_camera(self, cam_indx):
        # assert len(self.obj_tracking_camera_list) > 0
        return (
            self.obj_tracking_camera_list[
                cam_indx
            ].objectMarkTracker.get_tracking_object_id_list()
            if len(self.obj_tracking_camera_list) > 0
            else self.obj_tracking_camera_list
        )

    def get_gql_client(self):
        return self.gqlDataClient.get_client()

    def get_camera_list(self):
        return self.obj_tracking_camera_list

    def get_tracking_mode(self):
        return self.tracking_method

    def get_detection_method(self, input_method_string):
        for method in ObjectTrackingMethod:
            if method.value == input_method_string:
                return method
        # return default detection method
        return AppConfig.APP_TRACKING_METHOD

    class ObjectTrakcingCamera:
        def __init__(self):
            self.camera_name = None
            self.vcap = None
            self.mtx = None
            self.dist = None
            self.robot_name = None
            self.ROIMgr = None
            self.objectMarkTracker = None
            self.handeyeAutoMode = True
            self.handeye = None
            self.camObjOffset = None
