import multiprocessing as mp
import cv2
import numpy as np
import json
import base64
import sys
import socket
from websocket import create_connection, WebSocket
import threading

from applications.config.appconfig import AppConfig
from applications.calibcamera_engine import calibcamera_engine
from applications.objtracking_select import (
    process_application_selection,
)
from applications.etc.util import PrintMsg
from applications.bridge.bridge_data import (
    VideoBridgeData,
    ObjectBridgeData,
    ErrorBridgeData,
    BridgeDataType,
)


def thread_data_receive(sock, interproc_dict, command_queue):
    curr_thread = threading.currentThread()
    while getattr(curr_thread, "do_run", True):
        try:
            recv_message = sock.recv()
            if len(recv_message) > 0:
                recv_obj = json.loads(recv_message)
                (type, name, cmd) = (
                    recv_obj["type"],
                    recv_obj["name"],
                    recv_obj["body"],
                )
                PrintMsg.print_error(type, cmd)
                if type == BridgeDataType.CMD:
                    command_queue.put((name, cmd))
        except Exception as ex:
            PrintMsg.print_error(ex)

    PrintMsg.print_error("thread_data_receive ended")


def process_bridge_data(interproc_dict, ve, cq):
    ws = create_connection(
        f"ws://{AppConfig.ServerIP}:34000",
        sockopt=((socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),),
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

        try:
            # TODO: should modularize these data conversion..
            if interproc_dict["object"] != {}:
                name = interproc_dict["object"]["name"]
                object_data = interproc_dict["object"]["objectData"]
                bridge_data = ObjectBridgeData(name, object_data)
                ws.send(bridge_data.dumps())

            if interproc_dict["error"] != {}:
                name = interproc_dict["error"]["name"]
                message = interproc_dict["error"]["message"]
                bridge_data = ErrorBridgeData(name, message)
                ws.send(bridge_data.dumps())

            if interproc_dict["video"] != {}:
                # get width & height of the current frame
                name = interproc_dict["video"]["name"]
                width = interproc_dict["video"]["width"]
                height = interproc_dict["video"]["height"]
                frame = interproc_dict["video"]["frame"]

                if isinstance(frame, np.ndarray):
                    jpg_image = cv2.imencode(".jpg", frame)[1]
                    base64_encoded = base64.b64encode(jpg_image)

                    bridge_data = VideoBridgeData(
                        name,
                        base64_encoded.decode("utf8"),
                        width,
                        height,
                    )
                ws.send(bridge_data.dumps())

            # exit if 'app_exit' flag is set
            if interproc_dict["app_exit"] == True:
                break

        except Exception as e:
            PrintMsg.print_error(e)
            break

        interproc_dict["object"] = {}
        interproc_dict["video"] = {}
        interproc_dict["error"] = {}

    # close socket and terminate the recv thread
    ws_recv_thread.do_run = False
    ws.close()


def run_application_main(app_type, app_args):

    # start method with 'spawn'
    mp.set_start_method(method="spawn")

    # create an event for inte
    # r-process syncronization
    video_sync = mp.Event()

    # create command queue among processes
    cmd_queue = mp.Queue(maxsize=3)
    result_queue = mp.Queue(maxsize=3)

    # create a dictionary to exchange data among processes
    mp_manager = mp.Manager()
    mp_global_dict = mp_manager.dict()

    # node calls the select funciton with both application type and arguments
    mp_global_dict["app_type"] = app_type
    mp_global_dict["app_args"] = app_args
    mp_global_dict["app_exit"] = False

    # CAUTION: must define dictionary keys here
    mp_global_dict["video"] = dict()
    mp_global_dict["object"] = dict()
    mp_global_dict["error"] = dict()

    Processes = [
        mp.Process(
            target=process_application_selection,
            args=(
                mp_global_dict,
                video_sync,
                cmd_queue,
            ),
        ),
        mp.Process(
            target=process_bridge_data,
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
        PrintMsg.print_error("Invalid paramters..")
        sys.exit()

    run_application_main(sys.argv[1], sys.argv[2])
