import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco
import glob

# calibFile = cv2.FileStorage("calibData.xml", cv2.FILE_STORAGE_READ)
# cmnode = calibFile.getNode("cameraMatrix")
# mtx = cmnode.mat()
# dcnode = calibFile.getNode("distCoeff")
# dist = dcnode.mat()

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # operations on the frame
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters
        )

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

        # check if the ids list is not empty
        # if no check is added the code will crash
        if np.all(ids != None):

            # # estimate pose of each marker and return the values
            # # rvet and tvec-different from camera coefficients
            # rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
            # #(rvec-tvec).any() # get rid of that nasty numpy value array error

            # for i in range(0, ids.size):
            #     # draw axis for the aruco markers
            #     aruco.drawAxis(color_image, mtx, dist, rvec[i], tvec[i], 0.1)

            # draw a square around the markers
            aruco.drawDetectedMarkers(color_image, corners)

            # code to show ids of the marker found
            for i in range(0, ids.size):
                cv2.putText(
                    color_image,
                    str(ids[i]),
                    tuple(corners[i][0][0]),
                    font,
                    1,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
        else:
            # code to show 'No Ids' when no markers are found
            cv2.putText(
                color_image, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA
            )

        # display the resulting frame
        cv2.imshow("Image Show", color_image)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
except Exception as ex:
    print("Error :", ex, file=sys.stderr)

finally:
    # When everything done, release the capture
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
