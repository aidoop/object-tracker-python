import numpy as np

from visiongql.visiongql_data import VisonWorkspace, RobotArm, TrackingCamera, TrackableObject, VisionGqlUtil
from util.util import ObjectTypeCheck
from visionclient.operato_vision import Client


class VisonGqlDataClient:

    def __init__(self):
        self.visonWorkspace = None
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
        return (self.client.access_token is not None)

    def fetchTrackingCameras(self, cameras):
        for camera in cameras:
            name = camera['name']
            cameraData = self.client.get_tracking_camera(name=name)
            cameraObject = TrackingCamera()
            cameraObject.name = cameraData['name']
            cameraObject.baseRobotArm = cameraData['baseRobotArm']
            cameraObject.type = cameraData['type']
            cameraObject.endpoint = cameraData['endpoint']

            # no 6-elemets list
            tempList = cameraData['distortionCoefficient']
            if tempList is not None:    # check if gql data is not set yet..
                if len(tempList) > 5:
                    tempList.remove(tempList[5])
            cameraObject.distCoeff = np.array(tempList)

            # check if gql data is not set yet..
            if ObjectTypeCheck.checkValueIsAvail(cameraData['cameraMatrix']):
                cameraObject.setCameraMatrix(
                    cameraData['cameraMatrix']['rows'], cameraData['cameraMatrix']['columns'], cameraData['cameraMatrix']['data'])

            if ObjectTypeCheck.checkValueIsAvail(cameraData['handEyeMatrix']):
                cameraObject.setHandeyeMatrix(
                    cameraData['handEyeMatrix']['rows'], cameraData['handEyeMatrix']['columns'], cameraData['handEyeMatrix']['data'])

            if ObjectTypeCheck.checkValueIsAvail(cameraData['rois']):
                cameraObject.setROIData(cameraData['rois'])

            cameraObject.camObjOffset = VisionGqlUtil.setPoseData(
                cameraData['camObjOffset'])

            self.trackingCameras[name] = cameraObject

    def fetchTrackingCamerasAll(self):
        cameras = self.client.get_tracking_cameras()['items']
        self.fetchTrackingCameras(cameras)

    def fetchRobotArms(self, robotarms):
        for robotarm in robotarms:
            name = robotarm['name']
            robotarmData = self.client.get_robot_arm(name=name)
            robotarmObject = RobotArm()

            robotarmObject.name = robotarmData['name']
            robotarmObject.type = robotarmData['type']
            robotarmObject.endpoint = robotarmData['endpoint']
            robotarmObject.gripperOffset = VisionGqlUtil.setPoseData(
                robotarmData['toolOffset'])
            robotarmObject.markerOffset = VisionGqlUtil.setPoseData(
                robotarmData['markerOffset'])

            self.robotArms[name] = robotarmObject

    def fetchRobotArmsAll(self):
        robot_arms = self.client.get_robot_arms()['items']
        self.fetchRobotArms(robot_arms)

    def fetchTrackableMarks(self, marks):
        for mark in marks:
            name = mark['name']
            markData = self.client.get_trackable_object(name=name)
            markObject = TrackableObject()

            markObject.name = markData['name']
            markObject.type = markData['type']
            markObject.endpoint = markData['endpoint']
            markObject.poseOffset = VisionGqlUtil.setPoseData(
                markData['poiOffset'])

            self.trackableObjects[name] = markObject

    def fetchTrackableMarksAll(self):
        marks = self.client.get_trackable_objects()['items']
        self.fetchTrackableMarks(marks)

    # in case of only 1 workspace
    def fetchVisionWorkspace(self):
        wskpCnt = self.client.get_tracking_workspaces()['total']
        workspaces = self.client.get_tracking_workspaces()['items']
        if wskpCnt >= 2:
            print("one more workspaces found..")
            return (False, None)

        # fetch a vision workspace
        vision_workspace = workspaces[0]

        # create vision workspace class object
        self.visonWorkspace = VisonWorkspace()

        # set the properties of vision workspace
        self.visonWorkspace.name = vision_workspace['name']

        # fetch robot arms
        robot_arms = vision_workspace['robotArms']
        self.fetchRobotArms(robot_arms)

        # fetch cameras
        tracking_cameras = vision_workspace['trackingCameras']
        self.fetchTrackingCameras(tracking_cameras)

        # fetch trackable objects
        trackable_objects = vision_workspace['trackableObjects']
        self.fetchTrackableMarks(trackable_objects)

    def getRobotPose(self, name):
        return self.client.get_robot_arm_pose(name=name)
