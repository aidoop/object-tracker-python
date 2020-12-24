from abc import *
import enum

# CameraPortLayer abstraction class


class RobotArmDev(metaclass=ABCMeta):

    ###################################
    # Abstraction Functions
    ###################################
    # initialize parameters for any camera operation
    # TODO: manage robot parameters varied by robot manufacturer to connect a robot
    @abstractmethod
    def start(self, connect_ip):
        pass

    # finalize this object
    @abstractmethod
    def stop(self):
        pass

    # move-to on task cooridnate system
    @abstractmethod
    def move_task_to(self):
        pass

    # move-by on task cooridnate system
    @abstractmethod
    def move_task_by(self, xyzuvw):
        pass

    # move-by on task cooridnate system (no wait to finish moveing)
    @abstractmethod
    def move_task_by_async(self, xyzuvw):
        pass

    # joint-to on task cooridnate system
    @abstractmethod
    def joint_task_to(self, jointpos):
        pass

    # joint-by on task cooridnate system
    @abstractmethod
    def joint_task_by(self, jointpos):
        pass

    @abstractmethod
    def move_to_home(self):
        pass

    # go to zero
    @abstractmethod
    def move_to_zero(self):
        pass

    # get the current task position
    @abstractmethod
    def get_task_pos(self):
        pass

    # check next move for singularity
    @abstractmethod
    def check_next_move(self, next_task_pos, curr_joint_pos):
        pass

    # get the current joint position
    @abstractmethod
    def get_joint_pos(self):
        pass

    # set the direct-teaching mode
    @abstractmethod
    def set_teaching_mode(self, flag):
        pass

    # get the direct-teaching mode
    @abstractmethod
    def get_teaching_mode(self):
        pass

    # reset
    @abstractmethod
    def reset_robot(self):
        pass
