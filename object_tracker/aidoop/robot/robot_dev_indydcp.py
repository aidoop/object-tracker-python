from time import sleep

from aidoop.robot.robot_arm_dev import RobotArmDev
import aidoop.robot.provider.neuromeka.indydcp_client as indycli

# Neuromeka Indy7


class RobotIndy7Dev(RobotArmDev):

    # initialize parameters for robot operations
    # TODO: manage robot parameters varied by robot manufacturer to connect a robot
    def start(self, connect_ip):
        # connect to indy7 using fixed string "NRMK-Indy7"
        self.indy = indycli.IndyDCPClient(connect_ip, "NRMK-Indy7")
        conResult = self.indy.connect()
        if conResult == False:
            print("Connection Failed")
            obj = None
            return False

        self.indy.reset_robot()
        status = self.indy.get_robot_status()
        print("Resetting robot", file=sys.stderr)
        print("is in resetting? ", status["resetting"], file=sys.stderr)
        print("is robot ready? ", status["ready"], file=sys.stderr)
        if status["direct_teaching"] == True:
            self.indy.direct_teaching(False)
        if status["emergency"] == True:
            self.indy.stop_emergency()
        sleep(1)
        status = self.indy.get_robot_status()
        print("Reset robot done", file=sys.stderr)
        print("is in resetting? ", status["resetting"], file=sys.stderr)
        print("is robot ready? ", status["ready"], file=sys.stderr)
        return True

    # finalize this object
    def stop(self):
        self.indy.disconnect()

    # move to based on task coordinate system
    def move_task_to(self, xyzuvw):
        self.indy.task_move_to(xyzuvw)
        self.indy.wait_for_move_finish()

    # move by based on task coordinate system
    def move_task_by(self, xyzuvw):
        self.indy.task_move_by(xyzuvw)
        self.indy.wait_for_move_finish()

    # joint-to on task cooridnate system
    def joint_task_to(self, jointpos):
        self.indy.joint_move_to(jointpos)
        self.indy.wait_for_move_finish()

    # joint-by on task cooridnate system
    def joint_task_by(self, jointpos):
        self.indy.joint_move_by(jointpos)
        self.indy.wait_for_move_finish()

    # move by based on task coordinate system (no wait to finish moving)
    def move_task_by_async(self, xyzuvw):
        self.indy.task_move_by(xyzuvw)

    # get robot status
    def get_robot_status(self):
        return self.indy.get_robot_status()

    # go to home
    def move_to_home(self):
        self.indy.go_home()
        self.indy.wait_for_move_finish()

    # go to zero
    def move_to_zero(self):
        self.indy.go_zero()
        self.indy.wait_for_move_finish()

    # get the current hand position
    def get_task_pos(self):
        return self.indy.get_task_pos()

    def check_next_move(self, next_task_pos, curr_joint_pos):
        return self.indy.get_inv_kin(next_task_pos, curr_joint_pos)

    # get the current joint position
    def get_joint_pos(self):
        return self.indy.get_joint_pos()

    # set direct-teaching mode
    def set_teaching_mode(self, flag):
        self.indy.direct_teaching(flag)

    # get the direct-teaching mode
    def get_teaching_mode(self):
        robotStatus = self.indy.get_robot_status()
        return robotStatus["direct_teaching"]

    # reset the robot
    def reset_robot(self):
        self.indy.reset_robot()
