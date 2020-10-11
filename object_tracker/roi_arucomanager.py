import os
import numpy as np
from typing import NamedTuple
import cv2
import cv2.aruco as aruco

from aidobjtrack.config.appconfig import AppConfig
from aidobjtrack.aruco.aruco_detect import ArucoDetect

# TODO: should be derived in abstract class??


class ROIAruco2DManager:

    def __init__(self, markerSelectDict, markerSize, mtx, dist):
        # arouco marker id list
        self.arucoRangeList = []

        # ROI Region List
        self.ROIRegions = []
        self.ROIRegionIds = []

        # create an aruco detect object
        self.arucoDetect = ArucoDetect(markerSelectDict, markerSize, mtx, dist)

    def setMarkIdPair(self, arucoRangeData):
        self.arucoRangeList.append(arucoRangeData)

    def clearMakrIdPair(self):
        self.arucoRangeList.clear()

    def setROIRegion(self, pixelCoord1, pixelCoord2):
        self.ROIRegions.append((pixelCoord1, pixelCoord2))

    def setROIRegion(self, pixelCoord1, pixelCoord2, id):
        self.ROIRegions.append((pixelCoord1, pixelCoord2))
        self.ROIRegionIds.append(id)

    def getROIRegition(self):
        return self.ROIRegions

    def getROIRegionIDs(self):
        return self.ROIRegionIds

    def clearROIRegion(self):
        self.ROIRegions.clear()
        self.ROIRegionIds.clear()

    def printRangeData(self):
        for rdata in self.arucoRangeList:
            print('ID: ', self.ROIRegionIds[idx])
            print('Start: ' + str(rdata[0]) + ', End: ' + str(rdata[1]))

    def findROI(self, color_image, mtx, dist):
        # clear previous list
        self.clearROIRegion()

        # operations on the frame
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # lists of ids and the corners belonging to each id
        corners, ids = self.arucoDetect.detect(gray)

        if np.all(ids != None):
            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            rvec, tvec = self.arucoDetect.estimatePose(corners)

            centerPoint = dict()
            for arucoIDRange in self.arucoRangeList:
                if (arucoIDRange[0] in ids) and (arucoIDRange[1] in ids):
                    # get a retangle coordinates between two aruco marks
                    for arucoMark in arucoIDRange:
                        idx = list(ids).index(arucoMark)
                        inputObjPts = np.float32(
                            [[0.0, 0.0, 0.0]]).reshape(-1, 3)
                        imgpts, jac = cv2.projectPoints(
                            inputObjPts, rvec[idx], tvec[idx], mtx, dist)
                        centerPoint[arucoMark] = tuple(imgpts[0][0])

                    # set the current region to a list
                    self.setROIRegion(
                        centerPoint[arucoIDRange[0]], centerPoint[arucoIDRange[1]])

        return self.getROIRegition()

    def findROIPair(self, color_image, mtx, dist):
        # clear previous list
        self.clearROIRegion()

        # operations on the frame
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # lists of ids and the corners belonging to each id
        corners, ids = self.arucoDetect.detect(gray)

        if np.all(ids != None):
            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            rvec, tvec = self.arucoDetect.estimatePose(corners)

            for idx in range(len(ids)):
                for subidx in range(idx+1, len(ids)):
                    if(ids[idx] == ids[subidx]):
                        inputObjPts = np.float32(
                            [[0.0, 0.0, 0.0]]).reshape(-1, 3)
                        imgpts, jac = cv2.projectPoints(
                            inputObjPts, rvec[idx], tvec[idx], mtx, dist)
                        imgpts_sub, jac_sub = cv2.projectPoints(
                            inputObjPts, rvec[subidx], tvec[subidx], mtx, dist)

                        # set the current region to a list
                        self.setROIRegion(tuple(imgpts[0][0]), tuple(
                            imgpts_sub[0][0]), ids[idx][0])

            # centerPoint = dict()
            # for arucoIDRange in self.arucoRangeList:
            #     if (arucoIDRange[0] in ids) and (arucoIDRange[1] in ids):
            #         # get a retangle coordinates between two aruco marks
            #         for arucoMark in arucoIDRange:
            #             idx = list(ids).index(arucoMark)
            #             inputObjPts = np.float32([[0.0,0.0,0.0]]).reshape(-1,3)
            #             imgpts, jac = cv2.projectPoints(inputObjPts, rvec[idx], tvec[idx], mtx, dist)
            #             centerPoint[arucoMark] = tuple(imgpts[0][0])

            #         # set the current region to a list
            #         self.setROIRegion(centerPoint[arucoIDRange[0]], centerPoint[arucoIDRange[1]])

        return (self.getROIRegition(), self.getROIRegionIDs())


###############################################################################
# sample codes
###############################################################################
if __name__ == '__main__':
    pass
