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


class BridgeDataType:
    VIDEO = 'video'
    REQ = 'req'
    RES = 'res'
    DATA = 'data'


# class BridgeDataCreate:
#     @staticmethod
#     def create(type):
#         return VideoData()


class BridgeData:
    def __init__(self, type, name, message, width=0, height=0):
        self.json_data = dict()
        self.json_data['type'] = type
        self.json_data['name'] = name
        self.json_data['message'] = message
        self.json_data['width'] = width
        self.json_data['height'] = height

    def dumps(self):
        return json.dumps(self.json_data)

    def reset(self):
        self.json_data = {}


# IMPROVE-ME: check if you need any modification of frame level..
# class DataBridgeSocket(WebSocket):
#     def recv_frame(self):
#         recv_data = super().recv()
#         PrintMsg.printStdErr('recevied frame: ', frame)
#         return frame


def thread_data_receive(sock):
    curr_thread = threading.currentThread()
    while getattr(curr_thread, "do_run", True):
        recv_message = sock.recv()
        recv_obj = json.loads(recv_message)
        (type, cmd) = (recv_obj['type'], recv_obj['cmd'])
        print(type, cmd)


def proc_video_stream(interproc_dict, ve, dq):
    ws = create_connection("ws://localhost:34000",
                           sockopt=((socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),))

    ws_recv_thread = threading.Thread(target=thread_data_receive, args=(ws,))
    ws_recv_thread.start()

    while True:
        # wait for an video frame event
        ve.wait()

        # exit if 'app_exit' flast is set
        if interproc_dict['app_exit'] == True:
            break

        # get width & height of the current frame
        width = interproc_dict['video']['width']
        height = interproc_dict['video']['height']
        frame = interproc_dict['video']['frame']

        if isinstance(frame, np.ndarray):
            jpg_image = cv2.imencode('.jpg', frame)[1]
            base64_encoded = base64.b64encode(jpg_image)

            bridge_data = BridgeData(
                BridgeDataType.VIDEO, 'camera01', base64_encoded.decode('utf8'), width, height)

            try:
                ws.send(bridge_data.dumps())
            except Exception as e:
                PrintMsg.printStdErr(e)

    # make child thread finished
    ws_recv_thread.do_run = False


async def recv_msg(websocket, interproc_dict):
    recv_text = await websocket.recv()
    if recv_text == 'begin':
        while True:
            frame = interproc_dict['img']
            if isinstance(frame, np.ndarray):
                enconde_data = cv2.imencode('.jpg', frame)[1]
                enconde_str = enconde_data.tostring()
                try:
                    await websocket.send(enconde_str)
                except Exception as e:
                    PrintMsg.printStdErr(e)
                    return True


def image_put(q, camnum):
    cap = cv2.VideoCapture(camnum)
    i = 0
    while True:
        ret, frame = cap.read()
        if ret:
            frame = cv2.resize(frame, (500, 500))
            q.put(frame)
            # q.get() if q.qsize() > 1 else time.sleep(0.01)


def image_get(q, interproc_dict):
    while True:
        frame = q.get()
        if isinstance(frame, np.ndarray):
            interproc_dict['img'] = frame


def check_process_alive(mp_manager):
    pass
    # while True:
    #     PrintMsg.printStdErr(process.is_alive for process in)


def run_objtracking_engine(app_type, app_args):

    # TODO: check start method for multiprocessing spawn or ?
    mp.set_start_method(method='spawn')

    # TODO: check if event is a useful tool.
    video_sync = mp.Event()

    data_queue = mp.Queue(maxsize=10)

    mp_manager = mp.Manager()
    mp_global_dict = mp_manager.dict()

    # TODO: node calls the select funciton with both application type and arguments
    mp_global_dict['app_type'] = app_type
    mp_global_dict['app_args'] = app_args
    mp_global_dict['app_exit'] = False

    Processes = [
        mp.Process(target=objtracking_select_start,
                   args=(mp_global_dict, video_sync, data_queue, )),
        mp.Process(target=proc_video_stream,
                   args=(mp_global_dict, video_sync, data_queue, )),
    ]

    [process.start() for process in Processes]
    [process.join() for process in Processes]


if __name__ == '__main__':

    if len(sys.argv) != 3:
        PrintMsg.printStdErr("Invalid paramters..")
        sys.exit()

    run_objtracking_engine(sys.argv[1], sys.argv[2])
