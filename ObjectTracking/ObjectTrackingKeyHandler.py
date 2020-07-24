import cv2
import os
import glob
import numpy as np
import sys, os

# add src root directory to python path
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))) )
from packages.KeyHandler import KeyHandler

class ObjectTrackingKeyHandler(KeyHandler):

    def __init__(self):
        super().__init__()
        #super().setKeyHandler('q', self.processQ)     # should be exited by operato-ecs

    def processQ(self, *args):
        super().enableExitFlag()


    