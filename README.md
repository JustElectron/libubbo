# Libubbo

This is a C++ library for interfacing with the Ubbo Maker telepresence robot. It is intended to run on a linux platform, to be working with, but not limited to ROS. It has been developed and tested on a raspberry pi 3 b running ubuntu mate 16.04.
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
    sudo make install

### Pitfalls
Make sure that the serial library is installed in either /usr or /usr/local. This can be done by setting CMAKE_INSTALL_PREFIX using:
    
    cd serial
    cmake -DCMAKE_INSTALL_PREFIX=/usr
    make
    sudo make install
