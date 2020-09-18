import cv2
import os
import glob
import numpy as np
import sys
import os

from etc.keyhandler import KeyHandler


class ObjectTrackingKeyHandler(KeyHandler):

    def __init__(self):
        super().__init__()
        # super().setKeyHandler('q', self.processQ)     # should be exited by operato-ecs

    def processQ(self, *args):
        super().enableExitFlag()
