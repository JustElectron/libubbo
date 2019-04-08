#include <iostream>
#include <unistd.h>

#include "ubbo/ubbo.h"
#include "ubbo/commands.h"

int main(int argc, char** argv){
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "          Ubbo cpp serial test" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    ubbo::Ubbo my_ubbo("/dev/ttyACM0",9600);

    std::cout << "Serial port is:" << my_ubbo.getPort() << std::endl;

    if (my_ubbo.isConnected()){
        std::cout << "Serial port connected" << std::endl;
    }
    else {
        return 1;
    }

    usleep(5*1000*1000);

    my_ubbo.driveForward(0);
    usleep(1*1000*1000);
    my_ubbo.driveForward(45);
    usleep(1*1000*1000);
    my_ubbo.driveForward(90);
    usleep(1*1000*1000);
    my_ubbo.driveForward(135);
    usleep(1*1000*1000);
    my_ubbo.driveForward(180);
    usleep(1*1000*1000);
    my_ubbo.translateLeft();
    usleep(1*1000*1000);
    my_ubbo.translateRight();
    return 0;
}