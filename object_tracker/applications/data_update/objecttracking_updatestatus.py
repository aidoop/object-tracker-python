import sys
import os
import json
from random import choice, random

from applications.config.appconfig import AppConfig


class ObjectUpdateStatus(object):
    ObjStatusList = list()
    gqlClient = None

    def __init__(self, client):
        self.ObjStatusList.clear()
        self.gqlClient = client

    def addObjStatus(self, id, rois, x, y, z, u, v, w):
        for roi in rois:
            objStatus = {
                "id": str(id),
                "state": {
                    "roi": roi,
                    "pose": {"x": x, "y": y, "z": z, "u": u, "v": v, "w": w},
                },
            }
            self.ObjStatusList.append(objStatus)

    # don't send 'state' data for undetermined objects
    def addNoObjStatus(self, id):
        objStatus = {
            "id": str(id),
            # "state": {
            #     "roi": roi,
            #     "pose": {
            #         "x": x,
            #         "y": y,
            #         "z": z,
            #         "u": u,
            #         "v": v,
            #         "w": w
            #     }
            # }
        }
        self.ObjStatusList.append(objStatus)

    def sendObjStatus(self, undetectedObjectIDList):

        # process undetected marks
        for markerID in undetectedObjectIDList:
            # self.addObjStatus(markerID, [None],
            #                   None, None, None, None, None, None)
            self.addNoObjStatus(markerID)

        # process duplicated marks
        checkIDList = list()
        for objStatus in self.ObjStatusList:
            objID = objStatus["id"]

            if objID in checkIDList:
                self.ObjStatusList.remove(objStatus)
            else:
                checkIDList.append(objID)

        status = {"objectStatus": self.ObjStatusList}

        if AppConfig.ObjTrackingDebugMode == False:
            # print('-------------------')
            # print(status)
            self.gqlClient.update_tracking_workspace_status(
                name="workspace", status=status
            )
        else:
            print("\n", status, "\n")

    def containsObjStatus(self):
        return len(self.ObjStatusList) > 0

    def clearObjStatus(self):
        self.ObjStatusList.clear()
