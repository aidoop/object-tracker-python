import cv2
import cv2.aruco as aruco

import numpy as np

from ObjectTracker import ObjectTracker

# TODO: should be derived in a abstraction class like TrackingObject later.......
class ArucoMarkerObject:
    def __init__(self, markerID, pivotOffset):
        self.markerID = markerID
        self.pivotOffset = pivotOffset
        self.corners = None

    def print(self):
        print("hi")

# 
class ArucoMarkerTracker(ObjectTracker):
    def __init__(self):
        self.markerObjectList = []

    # initialize parameters for any camera operation
    def initialize(self, *args):
        self.markerSelectDict = args[0]
        self.markerSize = args[1]
        self.markerObjectList.clear()

    ## set detectable features like marker id of aruco marker
    def setTrackingObject(self, object):
        assert(isinstance(object, ArucoMarkerObject))
        self.markerObjectList.append(object)

    ## set detectable features and return the 2D or 3D positons in case that objects are detected..
    def findObjects(self, *args):
        color_image = args[0]
        mtx = args[1]
        dist = args[2]

        # prepare list to give over the result objects
        resultList = list()

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(self.markerSelectDict)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # set detected objects to the result list..
        if np.all(ids != None):
            for markerObject in self.markerObjectList:
                for idx in range(len(ids)):
                    if ids[idx] == markerObject.markerID:
                        # set additional properties to this found object..
                        print("Found Target ID: " + str(markerObject.markerID))
                        markerObject.corners = corners[idx].reshape(4,2)    # reshape: (1,4,2) --> (4,2)
                        
                        # append this object to the list to be returned
                        resultList.append(markerObject)

            # draw a square around the markers
            aruco.drawDetectedMarkers(color_image, corners)

        return resultList

        


        




    
