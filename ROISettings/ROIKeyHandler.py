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
            rur.addROIRegion("ROI" + str(idx), round(ROIRegions[idx][0][0]), round(ROIRegions[idx][0][1]), round(ROIRegions[idx][1][0]), round(ROIRegions[idx][1][1]))

        ROIRegionFile.release()
        print("ROI Regions saved.")

        rur.printROIRegions()






    

    