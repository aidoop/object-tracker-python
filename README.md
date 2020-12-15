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

- Module Installation
  - pip install -r requirements.txt (Aruco)
  - pip install -r requirements_mrcnn.txt (Box based on Mask-RCNN)
  


- Operato-Robotics (https://github.com/things-factory/things-factory/tree/master/packages/operato-robotics)

  - to run with Operato-Robotics, you should change paths in 'config/config.developmnet.ts' and 'config/config.production.ts' as below.
    The files should be copied to the root folder of 'operato-robotics' from 'operato-robotics/config' and then modified with the real path of 'object-tracker-python'

  ```
  module.exports = {
  vision: {
    streamingPort: 3001,
    camera: {
      cameraCalibrator: {
        program: [
          '/home/jinwon/Documents/venv/objtrack/bin/python3',
          '/home/jinwon/Documents/github/object-tracker-python/object_tracker/calibcamera_engine.py'
        ]
      },
      handEyeCalibrator: {
        program: [
          '/home/jinwon/Documents/venv/objtrack/bin/python3',
          '/home/jinwon/Documents/github/object-tracker-python/object_tracker/calibhandeye_engine.py'
        ]
      },
      ROIDetector: {
        program: [
          '/home/jinwon/Documents/venv/objtrack/bin/python3',
          '/home/jinwon/Documents/github/object-tracker-python/object_tracker/roi_engine.py'
        ]
      }
    },
    robotArm: {
      markerOffsetCalibrator: {
        program: [
          '/home/jinwon/Documents/venv/objtrack/bin/python3',
          '/home/jinwon/Documents/github/object-tracker-python/object_tracker/objecttracking_engine.py'
        ]
      }
    },
    objectTracker: {
      program: [
        '/home/jinwon/Documents/venv/objtrack/bin/python3',
        '/home/jinwon/Documents/github/object-tracker-python/object_tracker/objecttracking_engine.py'
      ]
    }
  }
  }
  ```

- References

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

  - how to get serial number of a realsense camera

  ```
  jinwon@jinwon-G5:~/Documents$ rs-sensor-control

  ======================================================

  Found the following devices:

    0 : Intel RealSense D435 #001622071306
    1 : Intel RealSense D435 #001622072547
  ```
