#include <iostream>
#include <unistd.h>
#include <chrono>
#include <math.h>

#include "ubbo/ubbo.h"
#include "ubbo/commands.h"

int main(int argc, char** argv){
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "          Ubbo cpp drive circle test" << std::endl;
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

    size_t data_send;
    double x = 0;
    int i = 0;
    while(i < 3){
        if (x > 360.0){
            x = 0.0;
            i++;
            
        }
        double velx = 0.4*sin(x*M_PI/180);
        double vely = 0.45*cos(x*M_PI/180);
        data_send = my_ubbo.drive(velx, vely, 0);
        std::cout << "data send: " << data_send << " drive: " << x << std::endl;
        if (data_send == 0){
            if (my_ubbo.isConnected()){
                std::cout << "Serial port connected" << std::endl;
                size_t size = my_ubbo.available();
                std::cout << "bytes available: " << unsigned(size) << std::endl;
                std::string buffer = my_ubbo.read(size);
                std::cout << buffer << std::endl;
            }
        }
        usleep(1*1000*500);
        x += 10;

    }

    return 0;
}