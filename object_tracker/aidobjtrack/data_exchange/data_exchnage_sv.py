import time
import multiprocessing as mp
import threading
from queue import Queue
import cv2
import numpy as np
import asyncio
import websockets
from websockets import ConnectionClosed


class DataExchangeServer:

    def __init__(self, url, port):
        self.url = url
        self.port = port
        self.ws_serv = None

    def start(self):
        try:
            if self.url is not None or self.port is not None:
                self.ws_serv = websockets.serve(
                    self.serv_handler, self.url, self.port)

                asyncio.get_event_loop().run_until_complete(self.ws_serv)
                asyncio.get_event_loop().run_forever()
        except Exception as exc:
            pass
        finally:
            asyncio.get_event_loop().run_until_complete(self.ws_serv)
            asyncio.get_event_loop().close()

    async def serv_handler(self, websocket, path):
        # TODO: add data exchange handler.
        name = await websocket.recv()
        print(f"< {name}")

        greeting = f"Hello {name}!"

        await websocket.send(greeting)
        print(f"> {greeting}")

    def stop(self):
        asyncio.get_event_loop.stop()


if __name__ == '__main__':

    dserv = DataExchangeServer("localhost", 8099)
    dserv.start()
