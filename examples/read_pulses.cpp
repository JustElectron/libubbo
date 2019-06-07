#include <iostream>
#include <unistd.h>
#include <chrono>
#include <math.h>

#include "ubbo/ubbo.h"
#include "ubbo/commands.h"


int main(int argc, char** argv){
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "          Ubbo cpp read thread test" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    ubbo::Ubbo my_ubbo("/dev/ttyACM0",57600);

    std::cout << "Serial port is:" << my_ubbo.getPort() << std::endl;

    if (my_ubbo.isConnected()){
        std::cout << "Serial port connected" << std::endl;
    }
    else {
        return 1;
    }

    usleep(5*1000*1000);

    std::cout << "Battery status: " << my_ubbo.getBatteryStatus() << std::endl;

    int count = 0;
    while (true){
        if (count < 70){
            size_t drive = my_ubbo.drive(0.4,0.0,0.0);
            //std::cout << "drive: " << drive << std::endl;
            count ++;
        }
            double pose_x = my_ubbo.position.x;
            double pose_y = my_ubbo.position.y;
            double rot = my_ubbo.position.yaw;
            std::cout << pose_x << "," << pose_y << "," << rot << std::endl;
            usleep(0.2*1000*1000);
    }
    return 0;
}