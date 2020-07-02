import json

class ObjectUpdateStatus:
    ObjStatusList = list()
    gqlClient = None

    def __init__(self, client):
        self.ObjStatusList.clear()
        self.gqlClient = client

    def addObjStatus(self, id, roi, x, y, z, u, v, w):
        objStatus = {
            "id": id,
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
        # TODO: change workspace name here.......
        #self.gqlClient.update_tracking_workspace_status(name='workspace', status=self.ObjStatusList)
        for ObjStatus in self.ObjStatusList:
            print('ObjStatus: ', ObjStatus)

    def containsObjStatus(self):
        return (len(self.ObjStatusList) > 0)




###############################################################################
# sample codes
###############################################################################

if __name__ == '__main__':
    rur = ROIUpdateRegions()

    rur.addROIRegion('abc', 100, 200, 500, 600)
    rur.addROIRegion('uuu', 300, 500, 600, 900)
    rur.printROIRegions()







