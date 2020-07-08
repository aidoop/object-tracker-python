import numpy as np

# add src root directory to python path
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from packages.VisionGqlData import VisonWorkspace, RobotArm, TrackingCamera, TrackableObject, VisionGqlUtil
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

    def fetchTrackingCameras(self):
        cameras = self.client.get_tracking_cameras()['items']

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
            if len(tempList) > 5:
                tempList.remove(tempList[5])
            cameraObject.distCoeff = np.array(tempList)
            
            cameraObject.setCameraMatrix(cameraData['cameraMatrix']['rows'], cameraData['cameraMatrix']['columns'], cameraData['cameraMatrix']['data'])
            cameraObject.setHandeyeMatrix(cameraData['handEyeMatrix']['rows'], cameraData['handEyeMatrix']['columns'], cameraData['handEyeMatrix']['data'])
            cameraObject.setROIData(cameraData['rois'])

            self.trackingCameras[name] = cameraObject

    def fetchRobotArms(self):
        robotarms = self.client.get_robot_arms()['items']

        for robotarm in robotarms:
            name = robotarm['name']
            robotarmData = self.client.get_robot_arm(name=name)
            robotarmObject = RobotArm()

            robotarmObject.name = robotarmData['name']
            robotarmObject.type = robotarmData['type']
            robotarmObject.endpoint = robotarmData['endpoint']
            robotarmObject.gripperOffset = VisionGqlUtil.setPoseData(robotarmData['toolOffset'])
            robotarmObject.markerOffset = VisionGqlUtil.setPoseData(robotarmData['markerOffset'])

            self.robotArms[name] = robotarmObject

    def fetchTrackableMarks(self):
        marks = self.client.get_trackable_objects()['items']

        for mark in marks:
            name = mark['name']
            markData = self.client.get_trackable_object(name=name)
            markObject = TrackableObject()

            markObject.name = markData['name']
            markObject.type = markData['type']
            markObject.endpoint = markData['endpoint']
            markObject.poseOffset = VisionGqlUtil.setPoseData(markData['poiOffset'])

            self.trackableObjects[name] = markObject

    # in case of only 1 workspace
    def fetchVisionWorkspace(self, client):
        workspaces = self.client.get_vision_workspaces()['items']
        if workspaces['total'] >= 2:
            print("one more workspaces found..")
            return (False, None)
        
        # fetch a vision workspace
        vision_workspace = workspaces[0]

        # create vision workspace class object
        self.visonWorkspace = VisonWorkspace()
        
        # set the properties of vision workspace
        self.visonWorkspace.name = vision_workspace['name']
        self.visonWorkspace.active = vision_workspace['active']
        if(self.visonWorkspace.active == False):
            return (False, None)
        
        # fetch robot arms
        robot_arms = vision_workspace['RobotArms']
        for robot_arm in robot_arms:
            robotArm = RobotArm()
            robotArm.name = robot_arm['name']
            robotArm.endpoint = robot_arm['endpoint']
            robotArm.ratype = robot_arm['type']
            robotArm.endpoint = robot_arm['endpoint']
            robotArm.active = robot_arm['active']
            robotArm.gripperOffset = [robot_arm['gripperOffset']['x'], robot_arm['gripperOffset']['y'], robot_arm['gripperOffset']['z']
                                     ,robot_arm['gripperOffset']['u'], robot_arm['gripperOffset']['v'], robot_arm['gripperOffset']['2']]
            self.visonWorkspace.robotArms[robotArm.name] = robotArm

        tracking_cameras = vision_workspace['TrackingCameras']
        for tracking_camera in tracking_cameras:
            trackingCamera = TrackingCamera()
            trackingCamera.name = tracking_camera['name']
            trackingCamera.endpoint = tracking_camera['endpoint']
            trackingCamera.tctype = tracking_camera['type']
            trackingCamera.active = tracking_camera['active']
            trackingCamera.config = tracking_camera['config']
            trackingCamera.baseRobotArm = tracking_camera['baseRobotArm']
            trackingCamera.cameraMatrix = tracking_camera['cameraMatrix']
            trackingCamera.handEyeMatrix = tracking_camera['handEyeMatrix']

            rois = tracking_camera['rois']
            for roi in rois:
                trackingCamera.rois[tracking_camera['rois']['id']] = [tracking_camera['rois']['id']['region']['tl']['x'], tracking_camera['rois']['id']['region']['tl']['y'], tracking_camera['rois']['id']['region']['br']['x'], tracking_camera['rois']['id']['region']['br']['y']]

            self.visonWorkspace.trackingCameras[trackingCamera.name] = trackingCamera
        
        trackable_objects = vision_workspace['TrackableObject']
        for trackable_object in trackable_objects:
            trackableObject = TrackableObject()
            trackableObject.name = trackable_object['name']
            trackableObject.poseOffset = [trackable_object['poseOffset']['x'], trackable_object['poseOffset']['y'], trackable_object['poseOffset']['z']
                                     ,trackable_object['poseOffset']['u'], trackable_object['poseOffset']['v'], trackable_object['poseOffset']['2']]
            trackableObject.active = trackable_object['active']
            trackableObject.totype = trackable_object['type']
            trackableObject.endpoint = trackable_object['endpoint']

            self.visonWorkspace.trackableObjects[trackableObject.name] = trackableObject



        
        


        
    
