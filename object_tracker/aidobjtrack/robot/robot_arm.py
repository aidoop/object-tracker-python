
import sys
import cv2

from aidobjtrack.camera.camera_dev_realsense import RealsenseCapture  # for an example


class RobotArm:

    def __init__(self, robot_dev, name):

        # set camera device object
        self.dev = robot_dev

        # set camera name
        self.name = name

    def start(self, connect_ip):
        # initialize camera device object
        return self.dev.start(connect_ip)

    def stop(self):
        return self.dev.stop()

    def get_name(self):
        return self.name

    def move_task_to(self, task_pos):
        return self.dev.move_task_to(task_pos)

    def move_task_by(self, task_pos):
        return self.dev.move_task_by(task_pos)

    def move_task_by_async(self, task_pos):
        return self.dev.move_task_by_async(task_pos)

    def joint_task_to(self, joint_pos):
        return self.dev.joint_task_to(joint_pos)

    def joint_task_by(self, joint_pos):
        return self.dev.joint_task_by(joint_pos)

    def get_robot_status(self):
        return self.dev.get_robot_status()

    def move_to_home(self):
        return self.dev.move_to_home()

    def move_to_zero(self):
        return self.dev.move_to_zero()

    def get_task_pos(self):
        return self.dev.get_task_pos()

    def get_joint_pos(self):
        return self.dev.get_joint_pos()

    def check_next_move(self, next_task_pos, curr_joint_pos):
        return self.dev.check_next_move(next_task_pos, curr_joint_pos)

    def set_teaching_mode(self, flag):
        return self.dev.set_teaching_mode(flag)

    def get_teaching_mode(self):
        return self.dev.get_teaching_mode()

    def reset_robot(self):
        return self.dev.reset_robot()


###############################################################################
# test sample codes
###############################################################################
if __name__ == '__main__':

    # create the camera object of intel realsense
    rsCamDev = RealsenseCapture(0)
    if(rsCamDev is None):
        print("Realsense device can't be opened..")
        sys.exit()

    # create video capture object using realsense camera object
    vcap = VideoCapture(rsCamDev, 1280, 720, 30, 'camera01')

    vcap.start()

    frameIdx = 0
    for frmIdx in range(0, 10):
        vcap.get_video_frame()
        print(frameIdx)
        frameIdx += 1

    vcap.stop()
