import sys
import cv2


class ObjectTypeCheck:
    @staticmethod
    def checkValueIsAvail(var):
        if var is None:
            return False
        return True


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
        # pass

# Info Text


class DisplayInfoText:

    def __init__(self, font, startPos):
        self.font = font
        self.startPos = startPos
        self.text = ''

    def draw(self, image):
        if self.text == '':
            return
        cv2.putText(image, self.text, self.startPos,
                    self.font, 1, (0, 255, 0), 1, cv2.LINE_AA)

    def setText(self, text):
        self.text = text
