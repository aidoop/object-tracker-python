import cv2
import os
import glob
import numpy as np

from packages.KeyHandler import KeyHandler
from ROIUpdateRegions import ROIUpdateRegions
from packages.Util import PrintMsg

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
        ROIRegionIds = args[2]

        regionCnt = len(ROIRegions)
        if regionCnt == 0:
            return

        savedFileName = "ROIRegions" + str(camIndex) + ".json" 
        ROIRegionFile = cv2.FileStorage(savedFileName, cv2.FILE_STORAGE_WRITE)
        regionCnt = len(ROIRegions)
        ROIRegionFile.write("ROICnt", regionCnt)

        rur = ROIUpdateRegions()

        for idx in range(regionCnt):
            ROIRegionFile.write("RegionID", str(ROIRegionIds[idx]))
            ROIRegionFile.write("Region", np.array(ROIRegions[idx]))
            rur.addROIRegion(str(ROIRegionIds[idx]), int(round(ROIRegions[idx][1][0])), int(round(ROIRegions[idx][1][1])), int(round(ROIRegions[idx][0][0])), int(round(ROIRegions[idx][0][1])))

        ROIRegionFile.release()
        PrintMsg.printStdErr("ROI Regions saved.")

        rur.printROIRegions()






    

    