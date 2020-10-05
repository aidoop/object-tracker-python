
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
from datetime import datetime

from camera import camera_dev_realsense, camera_videocapture
from visiongql import visiongql_client, visiongql_data
from robot import robot_dev_indydcp
from util.hm_util import HMUtil

from mrcnn import model as modellib, utils
from mrcnn.config import Config

import mrcnn_inference as rcnn


class GlobalData:
    vcap = None
    handeye = None
    robot = None


class InferenceConfig(Config):
    """Configuration for training on the toy  dataset.
    Derives from the base Config class and overrides some values.
    """
    # Give the configuration a recognizable name
    NAME = "object-train"

    # We use a GPU with 12GB memory, which can fit two images.
    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 1

    # Number of classes (including background)
    NUM_CLASSES = 1 + 1  # Background + balloon

    # Number of training steps per epoch
    STEPS_PER_EPOCH = 100

    # Skip detections with < 90% confidence
    DETECTION_MIN_CONFIDENCE = 0.9


def mouse_event_cb(event, x, y, flags, param):
    globalData = param

    if event == cv2.EVENT_LBUTTONDBLCLK:
        print('click: ', x, y)
        camera_coord = globalData.vcap.get3DPosition(x, y)
        print(camera_coord)

        hmTarget = HMUtil.makeHM(np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [
            0.0, 0.0, 1.0]]), np.array([camera_coord[0], camera_coord[1], camera_coord[2]+0.02]).T)

        hmResult = np.dot(globalData.handeye, hmTarget)
        print('result: ', hmResult)

        xyzuvw = HMUtil.convertHMtoXYZABCDeg(hmResult)

        [xn, yn, zn, un, vn, wn] = xyzuvw
        # xyzuvw = [x, y, z, u*(-1), v+180.0, w]
        curpos = glbData.robot.getCurrentPos()
        [xc, yc, zc, uc, vc, wc] = curpos
        xyzuvw = [xn, yn, zc, uc, vc, wc]
        print('last: ', xyzuvw)
        glbData.robot.moveTaskPos(xyzuvw)


if __name__ == '__main__':

    # create global data for test
    glbData = GlobalData()

    # open a realsense camera
    rsCamDev = camera_dev_realsense.RealsenseCapture('001622072547')
    if rsCamDev == None:
        print("can't initialize the realsense device")
        sys.exit()

    # connect gql server
    gqlDataClient = visiongql_client.VisonGqlDataClient()
    if(gqlDataClient.connect('http://localhost:3000', 'system', 'admin@hatiolab.com', 'admin') == False):
        print("Can't connect operato vision server.")
        sys.exit()
    gqlDataClient.fetchTrackingCamerasAll()

    # create an indy7 object
    indy7 = robot_dev_indydcp.RobotIndy7Dev()
    if(indy7.initalize("192.168.0.207", "NRMK-Indy7") == False):
        print("Can't connect the robot and exit this process..")
        sys.exit()
    glbData.robot = indy7

    # vistion tracking camera object list
    # vtcList = list()

    #########################################################################
    # intitialize tracking cameras
    idx = 0
    camKeys = gqlDataClient.trackingCameras.keys()
    for camKey in camKeys:
        # get gql camera information
        trackingCamera = gqlDataClient.trackingCameras[camKey]

        if(trackingCamera.endpoint != '10'):
            continue

        # set hand eye matrix
        glbData.handeye = trackingCamera.handEyeMatrix

    # set clipping distance
    # rsCamDev.setClipping(2.0)

    # set to apply post-filters
    rsCamDev.set_flag_filters(True)
    rsCamDev.prepare_filters()

    # create video capture object using realsense camera object
    vcap = camera_videocapture.VideoCapture(
        rsCamDev, 848, 480, 30, 'camera02')

    glbData.vcap = vcap

    vcap.start()

    # set mouse callback event
    cv2.namedWindow('projtest', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('projtest', mouse_event_cb, param=glbData)

    # initialize mask rcnn
    config = InferenceConfig()
    config.display()

    # create a model
    model = modellib.MaskRCNN(mode="inference", config=config,
                              model_dir="/home/jinwon/Documents/github/object-tracker-python/logs")

    # load weights
    model.load_weights(
        "/home/jinwon/Documents/github/object-tracker-python/mask_rcnn_nespresso_0030.h5", by_name=True)
    # "/home/jinwon/Documents/github/object-tracker-python/logs/object-train20201005T0817/mask_rcnn_object-train_0030.h5", by_name=True)

    # Streaming loop
    try:
        while True:

            # start: measure fps
            dt0 = datetime.now()

            (color_image, depth_image) = vcap.getFrame()

            if (color_image is None) or (depth_image is None):
                continue

            mask_list = rcnn.detect_object_by_data(model, color_image)
            for mask in mask_list:
                mask_image = np.where(mask, 255, 0).astype(np.uint8)
                cv2.imshow('mask', mask_image)
                key = cv2.waitKey(1000)

            # to view images using cv2.imshow
            color_image_view = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # render color & depth images
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                depth_image, alpha=0.03), cv2.COLORMAP_JET)
            images = np.hstack((color_image_view, depth_colormap))

            cv2.imshow('projtest', images)

            # start: measure fps
            process_time = datetime.now() - dt0
            # print("FPS: " + str(1 / process_time.total_seconds()))

            key = cv2.waitKey(1)
            # press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    finally:
        vcap.stop()
