
from time import sleep

from robot.robot_dev import RobotDev
import robot.indydcp_client as indycli
from util.util import PrintMsg

# Neuromeka Indy7


class RobotIndy7Dev(RobotDev):

    # initialize parameters for robot operations
    def initalize(self, servIP, connName):
        # Connect
        self.indy = indycli.IndyDCPClient(servIP, connName)
        conResult = self.indy.connect()
        if conResult == False:
            print("Connection Failed")
            obj = None
            return False

        self.indy.reset_robot()
        status = self.indy.get_robot_status()
        PrintMsg.printStdErr("Resetting robot")
        PrintMsg.printStdErr("is in resetting? ", status['resetting'])
        PrintMsg.printStdErr("is robot ready? ", status['ready'])
        if(status['direct_teaching'] == True):
            self.indy.direct_teaching(False)
        if(status['emergency'] == True):
            self.indy.stop_emergency()
        sleep(1)
        status = self.indy.get_robot_status()
        PrintMsg.printStdErr("Reset robot done")
        PrintMsg.printStdErr("is in resetting? ", status['resetting'])
        PrintMsg.printStdErr("is robot ready? ", status['ready'])
        return True

    # finalize this object
    def finalize(self):
        self.indy.disconnect()

    # start to move TCP
    def moveTaskPos(self, xyzuvw):
        self.indy.task_move_to(xyzuvw)
        self.indy.wait_for_move_finish()

    # get the current hand position
    def getCurrentPos(self):
        return self.indy.get_task_pos()

    def checkNextMove(self, next_task_pos, curr_joint_pos):
        return self.indy.get_inv_kin(next_task_pos, curr_joint_pos)

    # get the current joint position
    def getCurrentJointPos(self):
        return self.indy.get_joint_pos()

    # set direct-teaching mode
    def setDirectTeachingMode(self, flag):
        self.indy.direct_teaching(flag)

    # get the direct-teaching mode
    def getDirectTeachingMode(self):
        robotStatus = self.indy.get_robot_status()
        return robotStatus['direct_teaching']

    # reset
    def resetRobot(self):
        self.indy.reset_robot()
