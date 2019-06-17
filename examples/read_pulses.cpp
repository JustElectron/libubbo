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
        if (count < 150){
            size_t drive = my_ubbo.drive(0.0,-0.4,0.0);
            //std::cout << "drive: " << drive << std::endl;
            count ++;
        }
        else{
            my_ubbo.stop();
        }
            double pose_x = my_ubbo.position.x;
            double pose_y = my_ubbo.position.y;
            double rot = my_ubbo.position.yaw;

            float vel_x = my_ubbo.twist.linear_x;
            float vel_y = my_ubbo.twist.linear_y;
            float ang_rad_x = my_ubbo.twist.angular_z;
            
            std::cout << pose_x << "," << pose_y << "," << rot << ",";
            std::cout << vel_x << "," << vel_y << "," << ang_rad_x << std::endl;
            usleep(0.1*1000*1000);
    }
    return 0;
}
