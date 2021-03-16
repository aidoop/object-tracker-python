import json


class BridgeDataType:
    OBJECT = "object"
    VIDEO = "video"
    CMD = "cmd"


class BridgeData:
    BRIDGE_DATA = dict()

    def reset(self):
        BridgeData.BRIDGE_DATA.clear()


class VideoBridgeData(BridgeData):
    BRIDGE_DATA_TYPE = "video"

    def __init__(self, name, frame, width, height):
        self.name = name
        self.frame = frame
        self.width = width
        self.height = height

    def dumps(self):
        BridgeData.BRIDGE_DATA.clear()
        BridgeData.BRIDGE_DATA["name"] = self.name
        BridgeData.BRIDGE_DATA["body"] = {
            "type": VideoBridgeData.BRIDGE_DATA_TYPE,
            "frame": "data:image/jpeg;base64," + self.frame,
            "width": self.width,
            "height": self.height,
        }
        return json.dumps(BridgeData.BRIDGE_DATA)

    def reset(self):
        super.reset()


class ObjectBridgeData(BridgeData):
    BRIDGE_DATA_TYPE = "object"

    def __init__(self, name, object_data):
        self.name = name
        self.object_data = object_data

    def dumps(self):
        BridgeData.BRIDGE_DATA.clear()
        BridgeData.BRIDGE_DATA["name"] = self.name
        BridgeData.BRIDGE_DATA["body"] = {
            "type": ObjectBridgeData.BRIDGE_DATA_TYPE,
            "objectData": self.object_data,
        }
        return json.dumps(BridgeData.BRIDGE_DATA)

    def reset(self):
        super.reset()


class ErrorBridgeData(BridgeData):
    BRIDGE_DATA_TYPE = "error"

    def __init__(self, name, message):
        self.name = name
        self.message = message

    def dumps(self):
        BridgeData.BRIDGE_DATA.clear()
        BridgeData.BRIDGE_DATA["name"] = self.name
        BridgeData.BRIDGE_DATA["body"] = {
            "type": ErrorBridgeData.BRIDGE_DATA_TYPE,
            "message": self.message,
        }
        return json.dumps(BridgeData.BRIDGE_DATA)

    def reset(self):
        super.reset()
