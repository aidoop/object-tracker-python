
import numpy as np
import cv2
import cv2.aruco as aruco
import glob

calibFile = cv2.FileStorage("CalibCamResult10.json", cv2.FILE_STORAGE_READ)
cmnode = calibFile.getNode("cameraMatrix")
mtx = cmnode.mat()
dcnode = calibFile.getNode("distCoeff")
dist = dcnode.mat()

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
cap = cv2.VideoCapture(4)

# set opencv videocapture properties for width, height and fps
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)


def drawCube(image_frame):
    pass


while (True):
    ret, frame = cap.read()

    # operations on the frame
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

    # detector parameters can be set here (List of detection parameters[3])
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    print(ids)

    # font for displaying text (below)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # check if the ids list is not empty
    # if no check is added the code will crash
    if np.all(ids != None):

        # # estimate pose of each marker and return the values
        # # rvet and tvec-different from camera coefficients
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
            corners, 0.05, mtx, dist)
        # #(rvec-tvec).any() # get rid of that nasty numpy value array error

        # for i in range(0, ids.size):
        #     # draw axis for the aruco markers
        #     aruco.drawAxis(color_image, mtx, dist, rvec[i], tvec[i], 0.1)

        # draw a square around the markers
        aruco.drawDetectedMarkers(frame, corners)

        # code to show ids of the marker found
        for i in range(0, ids.size):
            cv2.putText(frame, str(ids[i]), tuple(
                corners[i][0][0]), font, 1, (0, 255, 0), 1, cv2.LINE_AA)

    else:
        # code to show 'No Ids' when no markers are found
        cv2.putText(frame, "No Ids", (0, 64), font,
                    1, (0, 255, 0), 2, cv2.LINE_AA)

    # display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
