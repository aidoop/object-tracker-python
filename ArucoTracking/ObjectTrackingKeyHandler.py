from KeyHandler import KeyHandler
import cv2
import os
import glob
import numpy as np

class ObjectTrackingKeyHandler(KeyHandler):

    def __init__(self):
        super().__init__()
        super().setKeyHandler('q', self.processQ)
        super().setKeyHandler('g', self.processG)

    def processQ(self, *args):
        super().enableExitFlag()

    def processG(self, *args):
        pass
    

    