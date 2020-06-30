import json

class ObjectUpdateStatus:
    ObjStatusList = list()
    gqlClient = None

    def __init__(self, client):
        self.ObjStatusList.clear()
        self.gqlClient = client

    def addObjStatus(self, id, x, y, z, u, v, w):
        objStatus = {
            "id": "obj",
            "state": {
                "roi": "A",
                "pose": {
                    "x": 1.0,
                    "y": 2.0,
                    "z": 3.4,
                    "u": 4.4,
                    "v": 5.9,
                    "w": 8.6
                }
            }
        }

        self.ObjStatusList.append(objStatus)

    def sendObjStatus(self):
        # TODO: change workspace name here.......
        result = client.update_tracking_workspace_status(name='workspace', status=self.ObjStatusList)

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







