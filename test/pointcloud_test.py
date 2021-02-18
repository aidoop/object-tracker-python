import numpy as np
import cv2
import cv2.aruco as aruco
import os
import time
import datetime
import glob
import sys
import argparse

import math
import pyrealsense2 as rs
import open3d

###############################################################################
# Hand-eye calibration process
#   -
###############################################################################

if __name__ == "__main__":

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 6)

    other_stream, other_format = rs.stream.color, rs.format.bgr8
    config.enable_stream(other_stream, 848, 480, other_format, 15)

    # Start streaming
    pipe_profile = pipeline.start(config)
    profile = pipeline.get_active_profile()

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Get stream profile and camera intrinsics
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    # Processing blocks
    pc = rs.pointcloud()
    decimate = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 2 ** 3)
    spatial = rs.spatial_filter()

    colorizer = rs.colorizer()
    filters = [
        rs.disparity_transform(),
        rs.spatial_filter(),
        rs.temporal_filter(),
        rs.disparity_transform(False),
    ]

    ## not aligned now
    # align_to = rs.stream.color
    # align = rs.align(align_to)

    try:
        while True:
            success, frames = pipeline.try_wait_for_frames(timeout_ms=0)
            if not success:
                continue

            depth_frame = frames.get_depth_frame().as_video_frame()
            other_frame = frames.first(other_stream).as_video_frame()

            depth_frame = decimate.process(depth_frame)

            if True:
                for f in filters:
                    depth_frame = f.process(depth_frame)

            color_image = np.asanyarray(other_frame.get_data())

            colorized_depth = colorizer.colorize(depth_frame)
            depth_colormap = np.asanyarray(colorized_depth.get_data())

            mapped_frame, color_source = other_frame, color_image
            # mapped_frame, color_source = colorized_depth, depth_colormap

            points = pc.calculate(depth_frame)
            pc.map_to(mapped_frame)

            np_points = np.random.rand(100, 3)

            # convert to open3d point cloud
            img_depth = Image(depth_colormap)
            img_color = Image(color_image)
            rgbd = create_rgbd_image_from_color_and_depth(
                img_color, img_depth, convert_rgb_to_intensity=False
            )

            # # focusing on a (320, 120) pixel
            # depth = depth_frame.get_distance(640, 480)
            # depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [320, 120], depth)
            # text = "%.5lf, %.5lf, %.5lf\n" % (depth_point[0], depth_point[1], depth_point[2])
            # print(text)

            # display the captured image
            cv2.imshow("PointCloud Test", color_image)

            # TODO: arrange these opencv key events based on other key event handler class
            # handle key inputs
            pressedKey = cv2.waitKey(1) & 0xFF
            if pressedKey == ord("q"):
                break

    except Exception as ex:
        print("Error :", ex, file=sys.stderr)

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()
