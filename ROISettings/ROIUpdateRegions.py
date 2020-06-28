import json

class ROIUpdateRegions:
    ROIRegionList = list()

    def __init__(self):
        self.ROIRegionList.clear()

    def addROIRegion(self, id, ltx, lty, rbx, rby):
        testNestedDict = {
            "id": id,
            "region": {
                "lt": {
                    "x": ltx,
                    "y": lty
                },
                "rb": {
                    "x": rbx,
                    "y": rby
                }
            }
        }

        self.ROIRegionList.append(testNestedDict)

    def printROIRegions(self):
        print(json.dumps(self.ROIRegionList))




###############################################################################
# sample codes
###############################################################################

if __name__ == '__main__':
    rur = ROIUpdateRegions()

    rur.addROIRegion('abc', 100, 200, 500, 600)
    rur.addROIRegion('uuu', 300, 500, 600, 900)
    rur.printROIRegions()







