import cv2
import os
import glob
import numpy as np

from packages.KeyHandler import KeyHandler
from ROIUpdateRegions import ROIUpdateRegions

class ROIKeyHandler(KeyHandler):

    def __init__(self):
        super().__init__()
        super().setKeyHandler('q', self.processQ)
        super().setKeyHandler('g', self.processG)

    def processQ(self, *args):
        super().enableExitFlag()

    def processG(self, *args):
        camIndex = args[0]
        ROIRegions = args[1]

        savedFileName = "ROIRegions" + str(camIndex) + ".json" 
        ROIRegionFile = cv2.FileStorage(savedFileName, cv2.FILE_STORAGE_WRITE)
        regionCnt = len(ROIRegions)
        ROIRegionFile.write("ROICnt", regionCnt)

        rur = ROIUpdateRegions()

        for idx in range(regionCnt):
            ROIRegionFile.write("ROI" + str(idx), np.array(ROIRegions[idx]))
            rur.addROIRegion("ROI" + str(idx), ROIRegions[0], ROIRegions[1], ROIRegions[2], ROIRegions[3])

        ROIRegionFile.release()
        print("ROI Regions saved.")

        rur.printROIRegions()






    

    