import numpy as np

class ROI:
    id = None
    tl = None
    rb = None

class PoseOffset:
    x = None
    y = None
    z = None
    u = None
    v = None
    w = None

## RobotArm
class RobotArm:
    id = None
    name = None
    type = None
    endpoint = None
    gripperOffset = None
    markerOffset = None

## Tracking Camera
class TrackingCamera:
    name = None
    type = None
    endpoint = None
    active = None
    config = None
    baseRobotArm = None
    distCoeff = None
    cameraMatrix = None
    handEyeMatrix = None
    ROIs = list()

    def setCameraMatrix(self, row, col, inputData):
        self.cameraMatrix = VisionGqlUtil.setMatrixData(row, col, inputData)
    
    def setHandeyeMatrix(self, row, col, inputData):
        self.setHandeyeMatrix = VisionGqlUtil.setMatrixData(row, col, inputData)        

    def _setMatrixData(self, row, col, inputData):
        if inputData is None:
            return
        return np.array(inputData).reshape(row, col)

    def setROIData(self, rois):
        if rois is None:
            return
        for roi in rois:
            if (roi['region']['lt'] is None) or (roi['region']['rb'] is None):
                continue
            roiObject = ROI()
            roiObject.id = roi['id']
            roiObject.tl = [roi['region']['lt']['x'], roi['region']['lt']['y']]
            roiObject.rb = [roi['region']['rb']['x'], roi['region']['rb']['y']]
            self.ROIs.append(roiObject)

## Trackable Object
class TrackableObject:
    name = None
    totype = None
    endpoint = None
    active = None
    poseOffset = None

## Workspace 
class VisonWorkspace:
    name = None
    wstype = None
    endpoint = None
    active = None
    params = None
    robotArms = dict()
    trackingCameras = dict()
    trackableObjects = dict()


# Utilities
class VisionGqlUtil:
    
    @staticmethod
    def setMatrixData(row, col, inputData):
        if inputData is None:
            return
        return np.array(inputData).reshape(row, col)

    @staticmethod
    def setPoseData(inputData):
        if inputData is None:
            return
        return np.array([inputData['x'], inputData['y'], inputData['z'], inputData['u'], inputData['v'], inputData['w']])

