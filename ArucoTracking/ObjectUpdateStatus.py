import json
from random import choice, random

class ObjectUpdateStatus:
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
                    "pose": {
                        "x": x,
                        "y": y,
                        "z": z,
                        "u": u,
                        "v": v,
                        "w": w
                    }
                }
            }
            self.ObjStatusList.append(objStatus)

    def sendObjStatus(self):
        status = {
        "objectStatus": self.ObjStatusList
        }
        self.gqlClient.update_tracking_workspace_status(name='workspace', status=status)
        print('\n', status, '\n')
        # status = {
        #     "objectStatus": [{
        #         "id": "obj",
        #         "state": {
        #             "roi": choice(["A", "B"]),
        #             "pose": {
        #                 "x": 1.0 + random() * 1.2,
        #                 "y": 2.2 + random() * 1.2,
        #                 "z": 3.4 + random() * 1.2,
        #                 "u": 4.4 + random() * 1.2,
        #                 "v": 7.9 + random() * 1.2,
        #                 "w": 9.6 + random() * 1.2
        #             }
        #         }
        #     }, {
        #         "id": "obj2",
        #         "state": {
        #             "roi": choice(["A", "B"]),
        #             "pose": {
        #                 "x": 1.0 + random() * 1.2,
        #                 "y": 19.0 + random() * 1.2,
        #                 "z": 4.4 + random() * 1.2,
        #                 "u": 9.4 + random() * 1.2,
        #                 "v": 21.9 + random() * 1.2,
        #                 "w": 13.6 + random() * 1.2
        #             }
        #         }
        #     }]
        # }
        # result = self.gqlClient.update_tracking_workspace_status(name='workspace', status=status)        


    def containsObjStatus(self):
        return (len(self.ObjStatusList) > 0)

    def clearObjStatus(self):
        self.ObjStatusList.clear()




###############################################################################
# sample codes
###############################################################################

if __name__ == '__main__':
    rur = ROIUpdateRegions()

    rur.addROIRegion('abc', 100, 200, 500, 600)
    rur.addROIRegion('uuu', 300, 500, 600, 900)
    rur.printROIRegions()







