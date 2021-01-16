import time
import multiprocessing as mp
import threading
from queue import Queue
import cv2
import numpy as np
import asyncio
import websockets
from websockets import ConnectionClosed


frame = None


def websocket_process(img_dict):
    # Server-side main logic
    async def main_logic(websocket, path):
        await recv_msg(websocket, img_dict)

    # new_loop = asyncio.new_event_loop()
    # asyncio.set_event_loop(new_loop)
    start_server = websockets.serve(main_logic, '0.0.0.0', 5678)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()


async def recv_msg(websocket, img_dict):
    recv_text = await websocket.recv()
    if recv_text == 'begin':
        while True:
            frame = img_dict['img']
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
            #q.get() if q.qsize() > 1 else time.sleep(0.01)


def image_get(q, img_dict):
    while True:
        frame = q.get()
        if isinstance(frame, np.ndarray):
            img_dict['img'] = frame


def run_single_camera(camnum):
    mp.set_start_method(method='spawn')  # init

    queue = mp.Queue(maxsize=3)
    m = mp.Manager()
    img_dict = m.dict()
    Processes = [mp.Process(target=image_put, args=(queue, camnum)),
                 mp.Process(target=image_get, args=(queue, img_dict)),
                 mp.Process(target=websocket_process, args=(img_dict, ))]

    [process.start() for process in Processes]
    [process.join() for process in Processes]


def run():
    run_single_camera(4)


if __name__ == '__main__':
    run()
