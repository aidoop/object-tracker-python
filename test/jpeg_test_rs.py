import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import io

from PIL import Image

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


try:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        raise Exception

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    im = Image.frombytes("RGB", (1280, 720), color_image)
    (b, g, r) = im.split()
    im = Image.merge("RGB", (r, g, b))

    # im.save('test.jpg')
    tempBuffer = io.BytesIO()
    im.save(tempBuffer, format="jpeg")
    aaaa = tempBuffer.getbuffer()

    print("printing jpeg header data")
    for idx in range(0, 128):
        print("")

    print("completed..")

except Exception as ex:
    print("Error :", ex, file=sys.stderr)

finally:
    # When everything done, release the capture
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
