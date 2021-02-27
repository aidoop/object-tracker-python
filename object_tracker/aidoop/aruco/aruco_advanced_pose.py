import cv2
import cv2.aruco as aruco

import numpy as np
import collections


class ArucoAdvPose:
    def __init__(self):

        self.isStable = False

        self.sizeEstPoses = 10
        self.queueEstimatedPoseX = collections.deque(maxlen=self.sizeEstPoses)
        self.queueEstimatedPoseY = collections.deque(maxlen=self.sizeEstPoses)
        self.queueEstimatedPoseZ = collections.deque(maxlen=self.sizeEstPoses)
        self.queueEstimatedPoseU = collections.deque(maxlen=self.sizeEstPoses)
        self.queueEstimatedPoseV = collections.deque(maxlen=self.sizeEstPoses)
        self.queueEstimatedPoseW = collections.deque(maxlen=self.sizeEstPoses)

        self.stdPoses = dict()

    def setPose(self, x, y, z, u, v, w):
        self.queueEstimatedPoseX.append(x)
        self.queueEstimatedPoseY.append(y)
        self.queueEstimatedPoseZ.append(z)
        self.queueEstimatedPoseU.append(u)
        self.queueEstimatedPoseV.append(v)
        self.queueEstimatedPoseW.append(w)

        self.stdPoses['x'] = np.std(self.queueEstimatedPoseX)
        self.stdPoses['y'] = np.std(self.queueEstimatedPoseY)
        self.stdPoses['z'] = np.std(self.queueEstimatedPoseZ)
        self.stdPoses['u'] = np.std(self.queueEstimatedPoseU)
        self.stdPoses['v'] = np.std(self.queueEstimatedPoseV)
        self.stdPoses['w'] = np.std(self.queueEstimatedPoseW)

        self.isStable = self.determineStable()

        # print('x std: ', self.stdPoses['x'])
        # print('y std: ', self.stdPoses['y'])
        # print('z std: ', self.stdPoses['z'])
        # print('u std: ', self.stdPoses['u'])
        # print('v std: ', self.stdPoses['v'])
        # print('w std: ', self.stdPoses['w'])
        # print('stable: ', self.isStable)

    def determineStable(self):
        return (self.stdPoses['x'] < 0.001) and (self.stdPoses['y'] < 0.001) and (self.stdPoses['z'] < 0.001) and (self.stdPoses['u'] < 0.5) and (self.stdPoses['v'] < 0.5) and (self.stdPoses['w'] < 0.5)

    def stable(self):
        return self.isStable

    def getPose(self, listPose):
        copyList = list(listPose)
        copyList.sort()
        return np.average(copyList[2:self.sizeEstPoses-2])

    def getPoses(self):

        if(len(self.queueEstimatedPoseX) < self.sizeEstPoses):
            return None

        avgPoses = dict()
        avgPoses['x'] = self.getPose(self.queueEstimatedPoseX)
        avgPoses['y'] = self.getPose(self.queueEstimatedPoseY)
        avgPoses['z'] = self.getPose(self.queueEstimatedPoseZ)
        avgPoses['u'] = self.getPose(self.queueEstimatedPoseU)
        avgPoses['v'] = self.getPose(self.queueEstimatedPoseV)
        avgPoses['w'] = self.getPose(self.queueEstimatedPoseW)

        return [avgPoses['x'], avgPoses['y'], avgPoses['z'], avgPoses['u'], avgPoses['v'], avgPoses['w']]
