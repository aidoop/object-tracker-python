# object-tracker-python

- Prerequisites
  - Python: > 3.x
  - Requried Python Modules
    - pip install numpy
    - pip install opencv-python
    - pip install opencv-contrib-python
    - pip install pyrealsense2
    - pip install requests
    - pip install gql
  - Install librealsense (https://github.com/IntelRealSense/librealsense/issues/793)
  ```
  git clone https://github.com/IntelRealSense/librealsense
  cd librealsense
  mkdir build
  cd build
  cmake ../ -DBUILD_PYTHON_BINDINGS=TRUE
  make -j4
  sudo make install #Optional if you want the library to be installed in your system
  ```

- vision-base (https://github.com/things-factory/vision-base.git)
  - to run with vision-base web application, you should change paths in 'vision-base/config.developmnet.ts' and 'vision-base/config.production.ts' as below.
  The files should be copied to the root folder of 'vision-base' from 'vision-base/config' and then modified with the real path of 'CoboMarkerTrakcing'
  ```
  module.exports = {
    vision: {
      streamingPort: 3001,
      camera: {
        cameraCalibrator: {
          program: ['/home/jinwon/Documents/venv/objtrack/bin/python3', '/home/jinwon/Documents/github/CoboMarkerTracking/CalibCamera/CalibCameraEngine.py']
        },
        handEyeCalibrator: {
          program: ['/home/jinwon/Documents/venv/objtrack/bin/python3', '/home/jinwon/Documents/github/CoboMarkerTracking/CalibHandEye/CalibHandEyeEngine.py']
        },
        ROIDetector: {
          program: ['/home/jinwon/Documents/venv/objtrack/bin/python3', '/home/jinwon/Documents/github/CoboMarkerTracking/ROISettings/ROISettingsEngine.py']
        }
      },
      robotArm: {
        markerOffsetCalibrator: {
          program: ['/home/jinwon/Documents/venv/objtrack/bin/python3', '/home/jinwon/Documents/github/CoboMarkerTracking/ArucoTracking/ObjectTrackingEngine.py']
        }
      },
      objectTracker: {
        program: ['/home/jinwon/Documents/venv/objtrack/bin/python3', '/home/jinwon/Documents/github/CoboMarkerTracking/ArucoTracking/ObjectTrackingEngine.py']
      }
    }
  }
  ```


- References
  - how to get serial number of a realsense camera
  ```
  jinwon@jinwon-G5:~/Documents$ rs-sensor-control

  ======================================================

  Found the following devices:

    0 : Intel RealSense D435 #001622071306
    1 : Intel RealSense D435 #001622072547
  ```
