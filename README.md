# CoboMarkerTracking

- Prerequisites
  - Python: > 3.6
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
