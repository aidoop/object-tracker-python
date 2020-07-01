import json

class CalibCameraUpdate:
    def __init__(self):
        pass

    def updateData(self, dist, cammtx):
        #resultList = list()

        jsonData = {
       "distortionCoefficient": [dist[0], dist[1], dist[2], dist[3], dist[4], 7.7],
        "cameraMatrix": {
            "rows": 3,
            "columns": 3,
            "data": [cammtx[0], cammtx[1], cammtx[2], cammtx[3], cammtx[4], cammtx[5], cammtx[6], cammtx[7], cammtx[8]]
        }
        }
        #resultList.append(testNestedDict)
        print(json.dumps(jsonData))

