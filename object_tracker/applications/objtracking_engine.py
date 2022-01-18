import cv2

import sys
import numpy as np
import traceback

from pyaidoop.log import Logger

from applications.config.appconfig import AppConfig, ObjectTrackingMethod
from applications.objtracking.objecttracking_app_data import ObjectTrakcingAppData
from applications.keyhandler.objecttracking_keyhandler import ObjectTrackingKeyHandler
from applications.data_update.objecttracking_updatestatus import ObjectUpdateStatus
from applications.etc.util import ObjectTrackerErrMsg

from applications.bridge.bridge_interprocess import BridgeInterprocess

objtracking_info = Logger.get("objtracking").info
objtracking_err = Logger.get("objtracking").error

###############################################################################
# Hand-eye calibration process
#   -
###############################################################################


def objecttracking_update_status(bridge_ip, current_status):
    bridge_ip.send_dict_data(
        "object",
        {
            "name": "objtracking",
            "objectData": current_status,
        },
    )


def objecttracking_alert_error(bridge_ip, error_msg):
    bridge_ip.send_dict_data(
        "error",
        {
            "name": "objtracking",
            "message": error_msg,
        },
    )


def objecttracking_engine(app_args, interproc_dict=None, ve=None, cq=None):

    objtracking_info("object tracking started..")

    bridge_ip = BridgeInterprocess(interproc_dict, ve, cq)

    objecttracking_update_status(bridge_ip, "Tracking Preparation Stage")

    # create application data
    try:
        #########################################################################
        #########################################################################
        # Tracking Preparation Stage
        #########################################################################
        #########################################################################
        objtracking_info("grpahql client parsing started..")
        app_data = ObjectTrakcingAppData()
        if (
            app_data.connect_server(
                f"http://{AppConfig.ServerIP}:3000",
                "system",
                "admin@hatiolab.com",
                "admin",
            )
            == False
        ):
            print("Can't connect operato vision server.")
            objtracking_err("grpahql client connection error..")
            sys.exit()

        app_data.parse()

        objecttracking_update_status(bridge_ip, "Preparing Data")

        #########################################################################
        # create key handler
        keyhandler = ObjectTrackingKeyHandler()

        #########################################################################
        # prepare object update status
        objtracking_info("object update status being created..")
        vision_workspace_name = (
            app_data.get_workspace().name
            if app_data.get_workspace() is not None
            else None
        )
        objStatusUpdate = ObjectUpdateStatus(
            app_data.get_gql_client(), vision_workspace_name
        )

        # if app_data.tracking_method == ObjectTrackingMethod.BOX:
        #     cams = app_data.get_camera_list()
        #     for cam in cams:
        #         cv2.namedWindow(cam.camera_name, cv2.WINDOW_NORMAL)
        #         cv2.setWindowProperty(
        #             cam.camera_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
        #         )

        objecttracking_update_status(bridge_ip, "Processing")
    except Exception as ex:
        objtracking_err(f"Prepartion stage error: {ex}")
        print(f"Prepartion stage error: {ex}")

    objtracking_info("Camera Frame Aqusition Stage")
    try:
        while True:

            #########################################################################
            #########################################################################
            # Camera Frame Aqusition Stage
            #########################################################################
            #########################################################################

            # get markable object
            tobjIDList = app_data.get_mark_id_list_of_camera(0).copy()

            vtcList = app_data.get_camera_list()
            if (
                ObjectTrackerErrMsg.check_value_zero(
                    len(vtcList), "the number of registered camera"
                )
                == False
            ):
                break

            vtc_image_index = 0
            vtc_image_buffers = list()
            for vtc in vtcList:
                # get a frame
                # color_image = vtc.vcap.get_video_frame()
                (color_image, depth_image) = vtc.vcap.get_frames()
                # assert color_image is not None, 'no captured frame'
                if color_image is None:
                    continue

                #########################################################################
                #########################################################################
                # Object Tracking Stage
                #########################################################################
                #########################################################################

                # check if core variables are available..
                if app_data.tracking_method == ObjectTrackingMethod.ARUCO:
                    if (
                        ObjectTrackerErrMsg.check_value_none(vtc.mtx, "camera matrix")
                        == False
                    ) or (
                        ObjectTrackerErrMsg.check_value_none(
                            vtc.dist, "distortion coeff."
                        )
                        == False
                    ):
                        break
                if (
                    ObjectTrackerErrMsg.check_value_none(
                        color_image, "video color frame"
                    )
                    == False
                ):
                    break
                if (
                    ObjectTrackerErrMsg.check_value_none(vtc.handeye, "hand eye matrix")
                    == False
                ):
                    raise Exception("Hand Eye Matrix is not found.")
                    break

                if AppConfig.ObjTrackingDebugWoRobot == False:
                    # find a robot arm related with the current camera.
                    raList = app_data.robot_arm_list
                    for ra in raList:
                        if ra.name == vtc.robot_name:
                            # detect markers here..
                            resultObjs = vtc.objectMarkTracker.find_tracking_object(
                                color_image, vtc, ra.gripperOffset, depth_image
                            )

                            if resultObjs == None:
                                continue

                            for resultObj in resultObjs:
                                # # TODO: manage ROI data later
                                # (found, foundRIDs) = vtc.ROIMgr.is_inside_roi(
                                #     resultObj.corners)
                                # found = False

                                if resultObj.targetPos is not None:
                                    [x, y, z, u, v, w] = resultObj.targetPos

                                    objStatusUpdate.add_tracking_object_status(
                                        resultObj.markerID,
                                        [vtc.camera_name],
                                        x,
                                        y,
                                        z,
                                        u,
                                        v,
                                        w,
                                    )

                                    # TODO: update object status based on ROIs later
                                    # if found is True:
                                    #     objStatusUpdate.add_tracking_object_status(
                                    #         resultObj.markerID, foundRIDs, x, y, z, u, v, w)
                                    # else:
                                    #     objStatusUpdate.add_tracking_object_status(
                                    #         resultObj.markerID, [None], x, y, z, u, v, w)
                                    if len(tobjIDList) > 0:
                                        tobjIDList.remove(resultObj.markerID)
                else:
                    # detect markers here..
                    resultObjs = vtc.objectMarkTracker.find_tracking_object(
                        color_image, vtc, None, depth_image
                    )

                    if resultObjs == None:
                        continue

                    for resultObj in resultObjs:
                        # # TODO: manage ROI data later
                        # (found, foundRIDs) = vtc.ROIMgr.is_inside_roi(
                        #     resultObj.corners)
                        # found = False

                        if resultObj.targetPos is not None:
                            [x, y, z, u, v, w] = resultObj.targetPos

                            # replace roi regions to camera name
                            objStatusUpdate.add_tracking_object_status(
                                resultObj.markerID, [vtc.camera_name], x, y, z, u, v, w
                            )

                            # TODO: update object status based on ROIs later
                            # if found is True:
                            #     objStatusUpdate.add_tracking_object_status(
                            #         resultObj.markerID, foundRIDs, x, y, z, u, v, w)
                            # else:
                            #     objStatusUpdate.add_tracking_object_status(
                            #         resultObj.markerID, [None], x, y, z, u, v, w)

                            if len(tobjIDList) > 0:
                                tobjIDList.remove(resultObj.markerID)

                # draw ROI Region..
                # for ROIRegion in vtc.ROIMgr.get_roi_list():
                #     cv2.rectangle(
                #         color_image, (ROIRegion[0], ROIRegion[1]), (ROIRegion[2], ROIRegion[3]), (255, 0, 0), 3)

                #########################################################################
                #########################################################################
                # Display Stage
                #########################################################################
                #########################################################################

                # display a frame image
                show_width = 640
                show_height = 360

                if app_data.tracking_method == ObjectTrackingMethod.ARUCO:
                    color_image_half = (
                        cv2.resize(
                            color_image,
                            (960, 540),
                        )
                        if AppConfig.VideoFrameWidth > 100
                        else color_image
                    )

                    vtc_image_buffers.append(color_image_half)
                    vtc_image_index += 1
                    # cv2.imshow(vtc.camera_name, color_image_half)
                elif app_data.tracking_method == ObjectTrackingMethod.BOX:
                    # BGR to RGB for opencv imshow function
                    color_image_view = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                    # create images to show
                    # sub_image = cv2.applyColorMap(cv2.convertScaleAbs(
                    #     depth_image, alpha=0.07), cv2.COLORMAP_JET)
                    sub_image = vtc.objectMarkTracker.get_mask_image(
                        color_image,
                        AppConfig.VideoFrameWidth,
                        AppConfig.VideoFrameHeight,
                    )
                    # sub_image = maskcv2.cvtColor(mask_image, cv2.COLOR_GRAY2RGB)

                    # draw pose axis in the mask image

                    # show scores
                    vtc.objectMarkTracker.put_info_text(color_image_view)

                    # display both color image and mask image
                    vtc_image_buffers.append(np.hstack((color_image_view, sub_image)))

                    show_width = 1280
                    show_height = 360
                elif app_data.tracking_method == ObjectTrackingMethod.PACK:
                    # BGR to RGB for opencv imshow function
                    color_image_view = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                    # get a depth image
                    depth_image_view = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_image, alpha=0.07), cv2.COLORMAP_JET
                    )

                    # get a color image
                    sub_image = vtc.objectMarkTracker.get_pickpoint_image(
                        color_image_view
                    )

                    # # show scores
                    # vtc.objectMarkTracker.put_info_text(color_image_view)

                    # display both color image and mask image
                    vtc_image_buffers.append(np.hstack((depth_image_view, sub_image)))

                    show_width = 1280
                    show_height = 360

                    # cv2.imshow(vtc.camera_name, images)

            #########################################
            # diplay or send image buffer
            if len(vtc_image_buffers) == 1:
                image_show_buffer = vtc_image_buffers[0]
            elif len(vtc_image_buffers) == 2:
                image_show_buffer = np.hstack(
                    (vtc_image_buffers[0], vtc_image_buffers[1])
                )
                (show_width, show_height) = (1280, 360)
            elif len(vtc_image_buffers) == 4:
                image_show_buffer_h1 = np.hstack(
                    (vtc_image_buffers[0], vtc_image_buffers[1])
                )
                image_show_buffer_h2 = np.hstack(
                    (vtc_image_buffers[2], vtc_image_buffers[3])
                )
                image_show_buffer = np.hstack(
                    (image_show_buffer_h1, image_show_buffer_h2)
                )
                (show_width, show_height) = (1280, 720)
            else:
                image_show_buffer = None

            # display the captured image
            if app_data.camera_streaming is True:
                if bridge_ip.isActive() is True:
                    color_image_resized = cv2.resize(
                        image_show_buffer,
                        dsize=(show_width, show_height),
                        interpolation=cv2.INTER_AREA,
                    )
                    bridge_ip.send_dict_data(
                        "video",
                        {
                            "name": "objtracking",
                            "width": show_width,
                            "height": show_height,
                            "frame": color_image_resized,
                        },
                    )
            else:
                cv2.imshow(
                    "Object Tracking", image_show_buffer
                ) if image_show_buffer is not None else None

            # send object information to UI and clear all
            # if objStatusUpdate.contains_object_status() == True:
            objStatusUpdate.sendObjStatus(tobjIDList)
            objStatusUpdate.clear_object_status()

            # sleep for the specified duration.
            # if app_data.tracking_method == ObjectTrackingMethod.ARUCO:
            #     time.sleep(0.2)

            #########################################################################
            #########################################################################
            # Key Event Process Stage
            #########################################################################
            #########################################################################

            # handle key inputs
            pressedKey = cv2.waitKey(1) & 0xFF
            if keyhandler.processKeyHandler(pressedKey):
                break

    except Exception as ex:
        objecttracking_alert_error(bridge_ip, f"Error: {ex}")
        print("Error :", ex, file=sys.stderr)
        objtracking_err(f"Main loop error: {ex}")
        print(traceback.format_exc(), file=sys.stderr)

    finally:

        if vtcList is not None:
            for vtc in vtcList:
                vtc.vcap.stop()

        objecttracking_update_status(bridge_ip, "Exit")
        bridge_ip.send_dict_data("app_exit", True)

        objtracking_info("object tracking ends..")

    cv2.destroyAllWindows()


if __name__ == "__main__":

    objecttracking_engine()
