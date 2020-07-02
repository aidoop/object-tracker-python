if __package__ == '':
    from RobotDev import RobotDev
    import indydcp_client as indycli
    from Util import PrintMsg
else:
    from packages.RobotDev import RobotDev
    import packages.indydcp_client as indycli
    from packages.Util import PrintMsg

from time import sleep


## Neuromeka Indy7
class RobotIndy7Dev(RobotDev):

    ## initialize parameters for robot operations
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
        if( status['direct_teaching'] == True):
            self.indy.direct_teaching(False)
        if( status['emergency'] == True):
            self.indy.stop_emergency()
        sleep(5)
        status = self.indy.get_robot_status()
        PrintMsg.printStdErr("Reset robot done")
        PrintMsg.printStdErr("is in resetting? ", status['resetting'])
        PrintMsg.printStdErr("is robot ready? ", status['ready'])
        return True

    ## finalize this object
    def finalize(self):
        self.indy.disconnect()

    ## start to move TCP
    def moveTaskPos(self, xyzuvw):
        self.indy.task_move_to(xyzuvw)

    ## get the current hand position
    def getCurrentPos(self):
        return self.indy.get_task_pos()

    ## set direct-teaching mode
    def setDirectTeachingMode(self, flag):
        self.indy.direct_teaching(flag)
    
    ## get the direct-teaching mode 
    def getDirectTeachingMode(self):
        robotStatus = self.indy.get_robot_status()
        return robotStatus['direct_teaching']

