import cv2
import cv2.aruco as aruco

import numpy as np
import collections


class ArucoAdvPose:
    def __init__(self):

        self.is_stable = False

        self.estimated_pose_size = 10
        self.queue_estimated_poseX = collections.deque(maxlen=self.estimated_pose_size)
        self.queue_estimated_poseY = collections.deque(maxlen=self.estimated_pose_size)
        self.queue_estimated_poseZ = collections.deque(maxlen=self.estimated_pose_size)
        self.queue_estimated_poseU = collections.deque(maxlen=self.estimated_pose_size)
        self.queue_estimated_poseV = collections.deque(maxlen=self.estimated_pose_size)
        self.queue_estimated_poseW = collections.deque(maxlen=self.estimated_pose_size)

        self.std_poses_dict = dict()

    def set_pose(self, x, y, z, u, v, w):
        self.queue_estimated_poseX.append(x)
        self.queue_estimated_poseY.append(y)
        self.queue_estimated_poseZ.append(z)
        self.queue_estimated_poseU.append(u)
        self.queue_estimated_poseV.append(v)
        self.queue_estimated_poseW.append(w)

        self.std_poses_dict["x"] = np.std(self.queue_estimated_poseX)
        self.std_poses_dict["y"] = np.std(self.queue_estimated_poseY)
        self.std_poses_dict["z"] = np.std(self.queue_estimated_poseZ)
        self.std_poses_dict["u"] = np.std(self.queue_estimated_poseU)
        self.std_poses_dict["v"] = np.std(self.queue_estimated_poseV)
        self.std_poses_dict["w"] = np.std(self.queue_estimated_poseW)

        self.is_stable = self.determin_stable()

        # print('x std: ', self.std_poses_dict['x'])
        # print('y std: ', self.std_poses_dict['y'])
        # print('z std: ', self.std_poses_dict['z'])
        # print('u std: ', self.std_poses_dict['u'])
        # print('v std: ', self.std_poses_dict['v'])
        # print('w std: ', self.std_poses_dict['w'])
        # print('stable: ', self.is_stable)

    def determin_stable(self):
        return (
            (self.std_poses_dict["x"] < 0.001)
            and (self.std_poses_dict["y"] < 0.001)
            and (self.std_poses_dict["z"] < 0.001)
            and (self.std_poses_dict["u"] < 0.5)
            and (self.std_poses_dict["v"] < 0.5)
            and (self.std_poses_dict["w"] < 0.5)
        )

    def stable(self):
        return self.is_stable

    def get_filtered_pose(self, listPose):
        copyList = list(listPose)
        copyList.sort()
        return np.average(copyList[2 : self.estimated_pose_size - 2])

    def get_filtered_poses(self):

        if len(self.queue_estimated_poseX) < self.estimated_pose_size:
            return None

        avg_poses_dict = dict()
        avg_poses_dict["x"] = self.get_filtered_pose(self.queue_estimated_poseX)
        avg_poses_dict["y"] = self.get_filtered_pose(self.queue_estimated_poseY)
        avg_poses_dict["z"] = self.get_filtered_pose(self.queue_estimated_poseZ)
        avg_poses_dict["u"] = self.get_filtered_pose(self.queue_estimated_poseU)
        avg_poses_dict["v"] = self.get_filtered_pose(self.queue_estimated_poseV)
        avg_poses_dict["w"] = self.get_filtered_pose(self.queue_estimated_poseW)

        return [
            avg_poses_dict["x"],
            avg_poses_dict["y"],
            avg_poses_dict["z"],
            avg_poses_dict["u"],
            avg_poses_dict["v"],
            avg_poses_dict["w"],
        ]
