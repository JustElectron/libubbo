# Libubbo

This is a C++ library for interfacing with the Ubbo Maker telepresence robot. It is supposed to be working with, but not limited to ROS
The project is created as a part of a bachelor project in electronics engineering. 

* Author: Andreas Just Rønning
* ROS driver: [ubbo_ros](https://github.com/JustElectron/ubbo_ros)

## Dependencies

* [serial](https://github.com/wjwwood/serial) cross-platrofm c++ library.
* [CMake](https://cmake.org/) Build system

### Build



    git clone https://github.com/JustElectron/libubbo.git
    cd libubbo
    mkdir build && cd build
    cmake ..
    make
