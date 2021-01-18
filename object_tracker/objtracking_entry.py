import time
import multiprocessing as mp
from queue import Queue
import cv2
import numpy as np
import asyncio
import websockets
from websockets import ConnectionClosed
from time import sleep

from calibcamera_engine import calibcamera_engine
from objtracking_select import objtracking_select_start, ObjectTrackingAppType, objtracking_select, app_tracking_data


def websocket_process(share_dict):
    # Server-side main logic
    async def main_logic(websocket, path):
        await recv_msg(websocket, share_dict)

    start_server = websockets.serve(main_logic, '0.0.0.0', 34567)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()


async def recv_msg(websocket, share_dict):
    recv_text = await websocket.recv()
    if recv_text == 'begin':
        while True:
            frame = share_dict['img']
            if isinstance(frame, np.ndarray):
                enconde_data = cv2.imencode('.png', frame)[1]
                enconde_str = enconde_data.tostring()
                try:
                    await websocket.send(enconde_str)
                except Exception as e:
                    print(e)
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


def image_get(q, share_dict):
    while True:
        frame = q.get()
        if isinstance(frame, np.ndarray):
            share_dict['img'] = frame


def check_process_alive(mp_manager):
    pass
    # while True:
    #     print(process.is_alive for process in)


def run_objtracking_engine():
    # TODO: check start method for multiprocessing
    mp.set_start_method(method='spawn')
    # TODO: check max queue size
    queue = mp.Queue(maxsize=3)

    mp_manager = mp.Manager()
    mp_global_dict = mp_manager.dict()
    app_tracking_data.set_dict(mp_global_dict)

    Processes = [  # mp.Process(target=image_put, args=(queue, camnum)),
        # mp.Process(target=image_get, args=(queue, mp_global_dict)),
        # mp.Process(target=calibcamera_engine, args=('camera02_rs', mp_global_dict)),
        mp.Process(target=objtracking_select_start, args=(mp_global_dict, )),
        mp.Process(target=websocket_process, args=(mp_global_dict, )),
    ]

    sleep(1)

    mp_global_dict['app_type'] = ObjectTrackingAppType.CAMERACALIB
    mp_global_dict['app_args'] = 'camera02_rs'

    [process.start() for process in Processes]
    [process.join() for process in Processes]


if __name__ == '__main__':
    run_objtracking_engine()
