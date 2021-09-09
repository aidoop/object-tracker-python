import cv2
import numpy as np
import sys
import math


from pyaidoop.calibration.calibhandeye_handeye import HandEyeCalibration
from pyaidoop.etc.hm_util import HMUtil
from applications.objtracking.objecttracking_base import ObjectTracker

from dl.odapi.detector import ObjectDetector


# tensorflow object detection api(ODApi) object
class ODApiObject:
    def __init__(self, markerID, pivotOffset):
        self.markerID = markerID
        self.pivotOffset = pivotOffset
        self.targetPos = None


# tensorflow object detection api(ODApi) object tracker
class ODApiObjectTracker(ObjectTracker):

    # a best candidate...
    MODEL_PATH = "/home/jinwon/Documents/github/pyaidoop-pickpoint-finder/exported_model/ssd_mobilenet_v2_kimchi_1/saved_model/"
    LABELMAP_PATH = "/home/jinwon/Documents/github/pyaidoop-pickpoint-finder/models/sample_labelmap.pbtxt"
    DETECTION_THRESHOLD = 0.7  # 0.5

    def __init__(self):
        # the list for input mark objects
        self.markerObjectList = []
        self.markerObjIDList = []

        # object addition data
        self.detected_objects = list()

        self.center_point_list = []
        self.scores_list = []
        self.box_list = []

        # pose information
        self.mask_rect_list = []
        self.mask_angle_list = []

    # initialize parameters for any camera operation

    def initialize(self, *args):
        self.handEyeMat = args[4]  # HandEyeCalibration.loadTransformMatrix()
        self.markerObjectList.clear()
        self.markerObjIDList.clear()

        self.detector = ObjectDetector(
            self.MODEL_PATH,
            self.LABELMAP_PATH,
            class_id=None,
            threshold=self.DETECTION_THRESHOLD,
        )

    # set detectable features like marker id of aruco marker
    def set_tracking_object(self, object):
        assert isinstance(object, ODApiObject)

        # TODO: handles two more object here?
        self.markerObjIDList.append(object.markerID)
        self.markerObjectList.append(object)

    def get_tracking_object_list(self):
        return self.markerObjectList

    def get_tracking_object_id_list(self):
        return self.markerObjIDList

    def get_lines_from_image(self, input_image):
        input_gray_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

        edge_image = cv2.Canny(input_gray_image, 75, 150)
        lines = cv2.HoughLinesP(edge_image, 1, np.pi / 180, 30, maxLineGap=300)
        return lines

    def find_optimal_line(self, lines, depth_image):
        line_depth_averages = list()
        min_line_depth_average = 9999999  # set an enougth big value
        for index, value in enumerate(lines):
            x1, y1, x2, y2 = value[0]
            line_iter = self.create_line_iterator((x1, y1), (x2, y2), depth_image)

            line_depth_average = 0
            line_depth_average_index = 0
            for line_point in line_iter:
                if line_point[2] != 0:
                    line_depth_average += line_point[2]
                    line_depth_average_index += 1
                # else:
                #    line_depth_average_index = 0
                #    break

            if line_depth_average_index > 0:
                line_depth_average = line_depth_average / line_depth_average_index

                # check the length of detected line is longer than minimun value either width or height of the extracted imate
                object_width, object_height = depth_image.shape
                min_line_length = int(
                    min(depth_image.shape[0], depth_image.shape[1]) / 2
                )
                if line_depth_average_index > min_line_length:
                    line_depth_averages.append(line_depth_average)
                else:
                    line_depth_averages.append(min_line_depth_average)
            else:
                line_depth_averages.append(min_line_depth_average)

            min_value = min(line_depth_averages)
            # print(min_value)
            min_index = line_depth_averages.index(min_value)
            # print(min_index)

        detected_line = lines[min_index]
        return detected_line

    # set detectable features and return the 2D or 3D positons in case that objects are detected..
    def find_tracking_object(self, *args):
        color_image = args[0]
        vtc = args[1]
        gripperOffset = args[2]
        depth_image = args[3]
        # prepare list to give over the result objects
        resultList = list()

        self.detected_objects.clear()

        ############################################################
        # detect objects
        if self.detector is not None:
            box_list = self.detector.detect_for_image(color_image)
            self.box_list = box_list

            for idx in range(len(box_list)):
                x_min = box_list[idx][0]
                y_min = box_list[idx][1]
                x_max = box_list[idx][2]
                y_max = box_list[idx][3]
                cls = str(box_list[idx][4])
                score = str(np.round(box_list[idx][-1], 2))

                object_image = color_image[y_min:y_max, x_min:x_max]
                object_depth_image = depth_image[y_min:y_max, x_min:x_max]

                lines = self.get_lines_from_image(object_image)

                line_depth_averages = list()
                min_line_depth_average = 9999999  # set an enougth big value
                for index, value in enumerate(lines):
                    x1, y1, x2, y2 = value[0]
                    line_iter = self.create_line_iterator(
                        (x1, y1), (x2, y2), object_depth_image
                    )

                    line_depth_average = 0
                    line_depth_average_index = 0
                    for line_point in line_iter:
                        if line_point[2] != 0:
                            line_depth_average += line_point[2]
                            line_depth_average_index += 1
                        # else:
                        #    line_depth_average_index = 0
                        #    break

                    if line_depth_average_index > 0:
                        line_depth_average = (
                            line_depth_average / line_depth_average_index
                        )

                        # check the length of detected line is longer than minimun value either width or height of the extracted imate
                        object_width, object_height = object_depth_image.shape
                        min_line_length = int(
                            min(
                                object_depth_image.shape[0], object_depth_image.shape[1]
                            )
                            / 2
                        )
                        if line_depth_average_index > min_line_length:
                            line_depth_averages.append(line_depth_average)
                        else:
                            line_depth_averages.append(min_line_depth_average)
                    else:
                        line_depth_averages.append(min_line_depth_average)

                    min_value = min(line_depth_averages)
                    # print(min_value)
                    min_index = line_depth_averages.index(min_value)
                    # print(min_index)

                detected_line = lines[min_index]
                x1_det, y1_det, x2_det, y2_det = detected_line[0]

                self.detected_objects.append(
                    {
                        "line": (
                            x1_det + x_min,
                            y1_det + y_min,
                            x2_det + x_min,
                            y2_det + y_min,
                        )
                    }
                )

            # TODO: go next
            # print(self.detected_objects)

            # # get the minArearect and angle for each mask
            # (self.mask_rect_list, self.mask_angle_list) = self.estimate_pose(mask_list)
            # print(self.mask_rect_list)
            # print(self.mask_angle_list)

            # self.center_point_list = self.find_box_center_list(self.mask_rect_list)
            # print(self.center_point_list)

            # self.scores_list = self.maskdetect.get_scores()
            # assert self.scores_list is not None or self.center_point_list is not None

            # for idx, (markerObject, cpoint, angle) in enumerate(
            #     zip(self.markerObjectList, self.center_point_list, self.mask_angle_list)
            # ):
            #     tvec = vtc.vcap.get_3D_pos(cpoint[0], cpoint[1])
            #     # print("---------------------------------------------")
            #     # print(vtc.vcap.get_3D_pos(cpoint[0], cpoint[1]))
            #     # print("---------------------------------------------")
            #     # NOTE: don't need to consider rotation here. we don't have any pose inforation except translation
            #     rvec = np.array([0.0, 0.0, 0.0])

            #     # change a rotation vector to a rotation matrix
            #     rotMatrix = np.zeros(shape=(3, 3))

            #     cv2.Rodrigues(rvec, rotMatrix)

            #     # make a homogeneous matrix using a rotation matrix and a translation matrix
            #     hmCal2Cam = HMUtil.create_homogeneous_matrix(rotMatrix, tvec)

            #     # fix z + 0.01 regardless of some input offsets like tool offset, poi offset,...
            #     # hmWanted = HMUtil.convert_xyzabc_to_hm_by_deg(
            #     #     offsetPoint) if offsetPoint is not None else np.eye(4)
            #     # hmInput = np.dot(hmCal2Cam, hmWanted)

            #     # get a final position
            #     hmResult = np.dot(self.handEyeMat, hmCal2Cam)
            #     xyzuvw_list = HMUtil.convert_hm_to_xyzabc_by_deg(hmResult)
            #     xyzuvw_arr = np.array(xyzuvw_list)

            #     offsetPoint = (
            #         markerObject.pivotOffset + vtc.camObjOffset
            #         if vtc.camObjOffset is not None
            #         else markerObject.pivotOffset
            #     )

            #     xyzuvw_arr += offsetPoint.tolist()
            #     xyzuvw = xyzuvw_arr.tolist()

            #     # this conversion is moved to Operato, but come back by making robot move policy consistent
            #     if xyzuvw != None:
            #         [x, y, z, u, v, w] = xyzuvw
            #         # indy7 base position to gripper position
            #         # xyzuvw = [x, y, z, u*(-1), v+180.0, w]
            #         # NOTE: using the fixed rotation information
            #         # TODO: adjust 'w' value by input angle value
            #         angle = angle + 180.0 if angle <= -90.0 else angle  # TEST
            #         xyzuvw = [x, y, z, 180.0, 0.0, 180.0 - angle]

            #     # set final target position..
            #     markerObject.targetPos = xyzuvw

            #     # append this object to the list to be returned
            #     resultList.append(markerObject)
        else:
            pass

        return resultList

    def get_scores_list(self):
        return self.scores_list

    def get_boxed_image(self, input_image):
        return self.detector.get_bboxes_on_image(input_image, self.box_list)

    def estimate_pose(self, mask_list):
        box_list = list()
        angle_list = list()

        if len(mask_list) > 0:
            for mask in mask_list:
                mask_image = np.where(mask, 255, 0).astype(np.uint8)
                contours, hierarchy = cv2.findContours(
                    mask_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
                )

                if len(contours) > 0:
                    for contour in contours:
                        # get minimum-area rect of each contour
                        areaRect = cv2.minAreaRect(contour)

                        # get rotation angle and push into the list
                        (rectWidth, rectHeight) = areaRect[1]
                        angle = (
                            (areaRect[2] - 90)
                            if (rectWidth < rectHeight)
                            else areaRect[2]
                        )
                        angle_list.append(angle)

                        # get the rectangle coordinates of RotateRect
                        box = cv2.boxPoints(areaRect)

                        # push into the list
                        box_list.append(box)

                else:
                    print("find mask but can not get any contour", file=sys.stderr)
        else:
            print("find mask in mask_list", file=sys.stderr)

        return (box_list, angle_list)

    def create_line_iterator(self, P1, P2, img):
        """
        Produces and array that consists of the coordinates and intensities of each pixel in a line between two points

        Parameters:
            -P1: a numpy array that consists of the coordinate of the first point (x,y)
            -P2: a numpy array that consists of the coordinate of the second point (x,y)
            -img: the image being processed

        Returns:
            -it: a numpy array that consists of the coordinates and intensities of each pixel in the radii (shape: [numPixels, 3], row = [x,y,intensity])
        """
        # define local variables for readability
        imageH = img.shape[0]
        imageW = img.shape[1]
        P1X = P1[0]
        P1Y = P1[1]
        P2X = P2[0]
        P2Y = P2[1]

        # difference and absolute difference between points
        # used to calculate slope and relative location between points
        dX = P2X - P1X
        dY = P2Y - P1Y
        dXa = np.abs(dX)
        dYa = np.abs(dY)

        # predefine numpy array for output based on distance between points
        itbuffer = np.empty(shape=(np.maximum(dYa, dXa), 3), dtype=np.float32)
        itbuffer.fill(np.nan)

        # Obtain coordinates along the line using a form of Bresenham's algorithm
        negY = P1Y > P2Y
        negX = P1X > P2X
        if P1X == P2X:  # vertical line segment
            itbuffer[:, 0] = P1X
            if negY:
                itbuffer[:, 1] = np.arange(P1Y - 1, P1Y - dYa - 1, -1)
            else:
                itbuffer[:, 1] = np.arange(P1Y + 1, P1Y + dYa + 1)
        elif P1Y == P2Y:  # horizontal line segment
            itbuffer[:, 1] = P1Y
            if negX:
                itbuffer[:, 0] = np.arange(P1X - 1, P1X - dXa - 1, -1)
            else:
                itbuffer[:, 0] = np.arange(P1X + 1, P1X + dXa + 1)
        else:  # diagonal line segment
            steepSlope = dYa > dXa
            if steepSlope:
                slope = dX.astype(np.float32) / dY.astype(np.float32)
                if negY:
                    itbuffer[:, 1] = np.arange(P1Y - 1, P1Y - dYa - 1, -1)
                else:
                    itbuffer[:, 1] = np.arange(P1Y + 1, P1Y + dYa + 1)
                itbuffer[:, 0] = (slope * (itbuffer[:, 1] - P1Y)).astype(np.int) + P1X
            else:
                slope = dY.astype(np.float32) / dX.astype(np.float32)
                if negX:
                    itbuffer[:, 0] = np.arange(P1X - 1, P1X - dXa - 1, -1)
                else:
                    itbuffer[:, 0] = np.arange(P1X + 1, P1X + dXa + 1)
                itbuffer[:, 1] = (slope * (itbuffer[:, 0] - P1X)).astype(np.int) + P1Y

        # Remove points outside of image
        colX = itbuffer[:, 0]
        colY = itbuffer[:, 1]
        itbuffer = itbuffer[
            (colX >= 0) & (colY >= 0) & (colX < imageW) & (colY < imageH)
        ]

        # Get intensities from img ndarray
        itbuffer[:, 2] = img[
            itbuffer[:, 1].astype(np.uint), itbuffer[:, 0].astype(np.uint)
        ]

        return itbuffer
