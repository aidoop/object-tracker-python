import time
import multiprocessing as mp
from queue import Queue
import cv2
import numpy as np
import json
import base64
import sys


import socket
from websocket import create_connection, WebSocket
import threading

from time import sleep

from calibcamera_engine import calibcamera_engine
from objtracking_select import objtracking_select_start, ObjectTrackingAppType
from aidobjtrack.util.util import PrintMsg


"""
Bridge Data Type
{
    type: bridge data type
    name: the specific application name
    body: json-style body data
}
"""


"""
Bridge Data Type
 - object
    name: object name
    body: varialbe json data
 - video
    name: camera name
    body: {frame: ..., width: xxx, height: yyy}
 - cmd
    name: command name
    body: {args: ...} (optional)
"""


class BridgeDataType:
    OBJECT = "object"
    VIDEO = "video"
    CMD = "cmd"


# class BridgeDataFactory:
#     @staticmethod
#     def create(type, name, body):


class BridgeData:
    BRIDGE_DATA = dict()

    def reset(self):
        BridgeData.BRIDGE_DATA = {}


class VideoBridgeData(BridgeData):
    def __init__(self, name, frame, width, height):
        self.type = BridgeDataType.VIDEO
        self.name = name
        self.frame = frame
        self.width = width
        self.height = height

    def dumps(self):
        BridgeData.BRIDGE_DATA = {}
        BridgeData.BRIDGE_DATA["type"] = self.type
        BridgeData.BRIDGE_DATA["name"] = self.name
        BridgeData.BRIDGE_DATA["body"] = {
            "frame": self.frame,
            "width": self.width,
            "height": self.height,
        }
        return json.dumps(BridgeData.BRIDGE_DATA)

    def reset(self):
        super.reset()


class ObjectBridgeData(BridgeData):
    def __init__(self, name, object):
        self.type = BridgeDataType.OBJECT
        self.name = name
        self.object = object

    def dumps(self):
        BridgeData.BRIDGE_DATA = {}
        BridgeData.BRIDGE_DATA["type"] = self.type
        BridgeData.BRIDGE_DATA["name"] = self.name
        BridgeData.BRIDGE_DATA["body"] = {
            "object": self.object,
        }
        return json.dumps(BridgeData.BRIDGE_DATA)

    def reset(self):
        super.reset()


# TODO: check if you need any modification of frame level..
# class DataBridgeSocket(WebSocket):
#     def recv_frame(self):
#         recv_data = super().recv()
#         PrintMsg.printStdErr('recevied frame: ', frame)
#         return frame


def thread_data_receive(sock, interproc_dict, command_queue):
    curr_thread = threading.currentThread()
    while getattr(curr_thread, "do_run", True):
        try:
            recv_message = sock.recv()
            if len(recv_message) > 0:
                recv_obj = json.loads(recv_message)
                (type, name, cmd) = (recv_obj["type"],
                                     recv_obj["name"], recv_obj["body"])
                PrintMsg.printStdErr(type, cmd)
                if type == BridgeDataType.CMD:
                    command_queue.put((name, cmd))
        except Exception as ex:
            PrintMsg.printStdErr(ex)

    PrintMsg.printStdErr("thread_data_receive ended")


def proc_video_stream(interproc_dict, ve, cq):
    ws = create_connection(
        "ws://localhost:34000", sockopt=((socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),)
    )

    ws_recv_thread = threading.Thread(
        target=thread_data_receive,
        args=(
            ws,
            interproc_dict,
            cq,
        ),
    )
    ws_recv_thread.start()

    while True:
        # wait for an video frame event
        ve.wait()

        # exit if 'app_exit' flast is set
        if interproc_dict["app_exit"] == True:
            break

        try:
            if interproc_dict["object"] != {}:
                object_name = interproc_dict["object"]["name"]
                object_data = interproc_dict["object"]["data"]
                bridge_data = ObjectBridgeData(name, object_data)
                ws.send(bridge_data.dumps())

            if interproc_dict["video"] != {}:
                # get width & height of the current frame
                device = interproc_dict["video"]["device"]
                width = interproc_dict["video"]["width"]
                height = interproc_dict["video"]["height"]
                frame = interproc_dict["video"]["frame"]

                if isinstance(frame, np.ndarray):
                    jpg_image = cv2.imencode(".jpg", frame)[1]
                    base64_encoded = base64.b64encode(jpg_image)

                    bridge_data = VideoBridgeData(
                        device,
                        base64_encoded.decode("utf8"),
                        width,
                        height,
                    )
                ws.send(bridge_data.dumps())

        except Exception as e:
            PrintMsg.printStdErr(e)
            break

        interproc_dict["object"] = {}
        interproc_dict["video"] = {}

    # close socket and terminate the recv thread
    ws_recv_thread.do_run = False
    ws.close()


def run_objtracking_engine(app_type, app_args):

    # TODO: check start method for multiprocessing spawn or ?
    mp.set_start_method(method="spawn")

    # TODO: check if event is a useful tool.
    video_sync = mp.Event()

    cmd_queue = mp.Queue(maxsize=3)

    mp_manager = mp.Manager()
    mp_global_dict = mp_manager.dict()

    # TODO: node calls the select funciton with both application type and arguments
    mp_global_dict["app_type"] = app_type
    mp_global_dict["app_args"] = app_args
    mp_global_dict["app_exit"] = False
    mp_global_dict["video"] = {}
    mp_global_dict["object"] = {}

    Processes = [
        mp.Process(
            target=objtracking_select_start,
            args=(
                mp_global_dict,
                video_sync,
                cmd_queue,
            ),
        ),
        mp.Process(
            target=proc_video_stream,
            args=(
                mp_global_dict,
                video_sync,
                cmd_queue,
            ),
        ),
    ]

    [process.start() for process in Processes]
    [process.join() for process in Processes]


if __name__ == "__main__":

    if len(sys.argv) != 3:
        PrintMsg.printStdErr("Invalid paramters..")
        sys.exit()

    run_objtracking_engine(sys.argv[1], sys.argv[2])
