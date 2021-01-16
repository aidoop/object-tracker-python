
import numpy as np
import cv2
import cv2.aruco as aruco
import glob

import base64

import asyncio
import websockets

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#cap = cv2.VideoCapture(cv2.CAP_DSHOW)
cap = cv2.VideoCapture(4)

# set opencv videocapture properties for width, height and fps
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

while (True):
    ret, frame = cap.read()

    # encode a video frame data
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    result, encoded_image = cv2.imencode('.jpg', frame, encode_param)
    content = encoded_image.tobytes()
    content_decode = base64.b64encode(content).decode('ascii')

    # display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
