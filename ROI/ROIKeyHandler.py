import cv2
import os
import glob
import numpy as np

from packages.KeyHandler import KeyHandler

class ROIKeyHandler(KeyHandler):

    def __init__(self):
        super().__init__()
        super().setKeyHandler('q', self.processQ)
        super().setKeyHandler('g', self.processG)

    def processQ(self, *args):
        super().enableExitFlag()

    def processG(self, *args):
        ROIRegions = args[0]

        ROIRegionFile = cv2.FileStorage("ROIRegions.json", cv2.FILE_STORAGE_WRITE)
        regionCnt = len(ROIRegions)
        ROIRegionFile.write("ROICnt", regionCnt)

        for idx in range(regionCnt):
            ROIRegionFile.write("ROI" + str(idx), np.array(ROIRegions[idx]))

        ROIRegionFile.release()

        print("ROI Regions saved.")

    

    