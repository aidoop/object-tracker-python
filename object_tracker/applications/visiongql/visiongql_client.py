import sys
import numpy as np

from applications.visiongql.visiongql_data import (
    VisonWorkspace,
    RobotArm,
    TrackingCamera,
    TrackableObject,
    VisionGqlUtil,
)
from applications.etc.util import ObjectTypeCheck
from pyaidoop_graphql_client.api import Client


class VisonGqlDataClient:
    def __init__(self):
        self.visonWorkspace = None
        self.checkVideoStream = None
        self.detectionMethod = None
        self.robotArms = dict()
        self.trackingCameras = dict()
        self.trackableObjects = dict()

        self.client = None

    def connect(self, url, domain, id, pw):
        """
        Operato 서버에 인증 과정 API
        """
        self.client = Client(url, domain)
        self.client.signin(id, pw)
        return self.client.access_token is not None

    def get_client(self):
        return self.client

    def fetch_tracking_cameras(self, cameras):
        for camera in cameras:
            name = camera["name"]
            cameraData = self.client.get_tracking_camera(name=name)
            cameraObject = TrackingCamera()
            cameraObject.name = cameraData["name"]
            cameraObject.baseRobotArm = cameraData["baseRobotArm"]
            cameraObject.type = cameraData["type"]
            cameraObject.endpoint = cameraData["endpoint"]

            # no 6-elemets list
            tempList = cameraData["distortionCoefficient"]
            if tempList is not None:  # check if gql data is not set yet..
                if len(tempList) > 5:
                    tempList.remove(tempList[5])
                cameraObject.distCoeff = np.array(tempList)

            # check if gql data is not set yet..
            if ObjectTypeCheck.check_value_available(cameraData["cameraMatrix"]):
                cameraObject.setCameraMatrix(
                    cameraData["cameraMatrix"]["rows"],
                    cameraData["cameraMatrix"]["columns"],
                    cameraData["cameraMatrix"]["data"],
                )

            # deleted handEyeAutoMode
            # cameraObject.handEyeAutoMode = cameraData['handEyeAutoMode']

            if ObjectTypeCheck.check_value_available(cameraData["handEyeMatrix"]):
                cameraObject.setHandeyeMatrix(
                    cameraData["handEyeMatrix"]["rows"],
                    cameraData["handEyeMatrix"]["columns"],
                    cameraData["handEyeMatrix"]["data"],
                )

            if ObjectTypeCheck.check_value_available(cameraData["rois"]):
                cameraObject.setROIData(cameraData["rois"])

            cameraObject.camObjOffset = VisionGqlUtil.setPoseData(
                cameraData["camObjOffset"]
            )

            cameraObject.width = (
                int(cameraData["width"])
                if (cameraData["width"] is not None) and (cameraData["width"] is not "")
                else 1920
            )
            cameraObject.height = (
                int(cameraData["height"])
                if (cameraData["height"] is not None)
                and (cameraData["height"] is not "")
                else 1080
            )

            cameraObject.handEyeMode = (
                cameraData["handEyeConfig"]["mode"]
                if cameraData["handEyeConfig"] is not None
                else "hand-to-eye"
            )

            cameraObject.autoHandeyeTotalIterations = (
                int(cameraData["handEyeConfig"]["autoTotalIter"])
                if cameraData["handEyeConfig"] is not None
                else 30
            )
            cameraObject.autoHandeyeMoveXyz = (
                float(cameraData["handEyeConfig"]["autoMoveXyz"])
                if cameraData["handEyeConfig"] is not None
                else 0.05
            )
            cameraObject.autoHandeyeMoveUvw = (
                float(cameraData["handEyeConfig"]["autoMoveUvw"])
                if cameraData["handEyeConfig"] is not None
                else 10
            )

            self.trackingCameras[name] = cameraObject

    def fetch_tracking_camera_all(self):
        cameras = self.client.get_tracking_cameras()["items"]
        self.fetch_tracking_cameras(cameras)

    def fetch_robot_arms(self, robotarms):
        for robotarm in robotarms:
            name = robotarm["name"]
            robotarmData = self.client.get_robot_arm(name=name)
            robotarmObject = RobotArm()

            robotarmObject.name = robotarmData["name"]
            robotarmObject.type = robotarmData["type"]
            robotarmObject.endpoint = robotarmData["endpoint"]
            robotarmObject.gripperOffset = VisionGqlUtil.setPoseData(
                robotarmData["toolOffset"]
            )
            robotarmObject.markerOffset = VisionGqlUtil.setPoseData(
                robotarmData["markerOffset"]
            )

            self.robotArms[name] = robotarmObject

    def fetch_robot_arm_all(self):
        robot_arms = self.client.get_robot_arms()["items"]
        self.fetch_robot_arms(robot_arms)

    def fetch_trackable_objects(self, marks):
        for mark in marks:
            name = mark["name"]
            markData = self.client.get_trackable_object(name=name)
            markObject = TrackableObject()

            markObject.name = markData["name"]
            markObject.type = markData["type"]
            markObject.endpoint = markData["endpoint"]
            markObject.poseOffset = VisionGqlUtil.setPoseData(markData["poiOffset"])

            self.trackableObjects[name] = markObject

    def fetch_trackable_objects_all(self):
        marks = self.client.get_trackable_objects()["items"]
        self.fetch_trackable_objects(marks)

    # in case of only 1 workspace
    def fetch_vision_workspace(self):
        wskpCnt = self.client.get_tracking_workspaces()["total"]
        workspaces = self.client.get_tracking_workspaces()["items"]
        if wskpCnt >= 2:
            print("one more workspaces found..", file=sys.stderr)
            return (False, None)

        # fetch a vision workspace
        vision_workspace = workspaces[0]

        # create vision workspace class object
        self.visonWorkspace = VisonWorkspace()

        # set the properties of vision workspace
        self.visonWorkspace.name = vision_workspace["name"]

        # fetch video stream
        self.checkVideoStream = vision_workspace["checkVideoStream"]

        # fetch detection methond
        self.detectionMethod = vision_workspace["detectionMethod"]

        # fetch robot arms
        robot_arms = vision_workspace["robotArms"]
        self.fetch_robot_arms(robot_arms)

        # fetch cameras
        tracking_cameras = vision_workspace["trackingCameras"]
        self.fetch_tracking_cameras(tracking_cameras)

        # fetch trackable objects
        trackable_objects = vision_workspace["trackableObjects"]
        self.fetch_trackable_objects(trackable_objects)

    def get_robot_pose(self, name):
        return self.client.get_robot_arm_pose(name=name)

    def move_robotarm_task_by(self, name, pose):
        return self.client.robot_task_moveby(name=name, pose=pose)

    def move_robotarm_task_by_nowait(self, name, pose):
        return self.client.robot_task_moveby_nowait(name=name, pose=pose)

    def get_robot_status(self, name):
        return self.client.get_robot_status(name=name)
