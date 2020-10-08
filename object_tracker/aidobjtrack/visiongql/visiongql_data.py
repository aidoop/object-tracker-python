import numpy as np


class ROI:

    def __init__(self):
        self.id = None
        self.tl = None
        self.rb = None

# RobotArm


class RobotArm:

    def __init__(self):
        self.id = None
        self.name = None
        self.type = None
        self.endpoint = None
        self.gripperOffset = None
        self.markerOffset = None

# Tracking Camera


class TrackingCamera:

    def __init__(self):
        self.ROIs = list()
        self.name = None
        self.type = None
        self.endpoint = None
        self.active = None
        self.config = None
        self.baseRobotArm = None
        self.distCoeff = None
        self.cameraMatrix = None
        self.handEyeMatrix = None
        self.camObjOffset = None

    def setCameraMatrix(self, row, col, inputData):
        self.cameraMatrix = VisionGqlUtil.setMatrixData(row, col, inputData)

    def setHandeyeMatrix(self, row, col, inputData):
        self.handEyeMatrix = VisionGqlUtil.setMatrixData(row, col, inputData)

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

# Trackable Object


class TrackableObject:

    def __init__(self):
        self.name = None
        self.totype = None
        self.endpoint = None
        self.active = None
        self.poseOffset = None

# Workspace


class VisonWorkspace:

    def __init__(self):
        self.name = None
        self.wstype = None
        self.endpoint = None
        self.active = None
        self.params = None

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
