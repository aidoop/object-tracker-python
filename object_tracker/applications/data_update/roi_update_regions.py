import json


class ROIUpdateRegions:
    roi_region_list = list()

    def __init__(self):
        self.roi_region_list.clear()

    def add_roi_region(self, id, ltx, lty, rbx, rby):
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

        self.roi_region_list.append(testNestedDict)

    def print_roi_regions(self):
        print(json.dumps(self.roi_region_list))


###############################################################################
# sample codes
###############################################################################
if __name__ == '__main__':
    rur = ROIUpdateRegions()

    rur.add_roi_region('abc', 100, 200, 500, 600)
    rur.add_roi_region('uuu', 300, 500, 600, 900)
    rur.print_roi_regions()
