import sys

class ArucoTrackerErrMsg:
    @staticmethod
    def checkValueIsNone(var, varname):
        if var is None:
            print("Error: " + varname + " is not allocated..")
            return False
        return True
       

class PrintMsg:
    @staticmethod
    def printStdErr(*args, **kwargs):
        return print(*args, **kwargs, file=sys.stderr)
        #pass


if __name__ == '__main__':
    var1 = None
    # check core variables are available..
    print(ArucoTrackerErrMsg.checkValueIsNone(var1, 'test variable1') == False)
        

