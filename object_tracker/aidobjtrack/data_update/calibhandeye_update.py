import json


class CalibHandeyeUpdate:
    def __init__(self):
        pass

    def updateData(self, handeye):

        jsonData = {
            "rows": 4,
            "columns": 4,
            "data": [handeye[0], handeye[1], handeye[2], handeye[3], handeye[4], handeye[5], handeye[6], handeye[7],
                     handeye[8], handeye[9], handeye[10], handeye[11], handeye[12], handeye[13], handeye[14], handeye[15]]
        }
        # resultList.append(testNestedDict)
        print(json.dumps(jsonData))
