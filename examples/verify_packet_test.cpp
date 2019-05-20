#include <iostream>
#include <unistd.h>
#include <chrono>

#include "ubbo/ubbo.h"
#include "ubbo/commands.h"


void print_vector(std::vector<uint8_t> const& input){
    for (int i = 0; i < input.size(); i++){
        std::cout << (unsigned)input.at(i) << ' ';
    }
}

int main(int argc, char** argv){
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "          Ubbo cpp serial test" << std::endl;
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

    int count = 0;
    my_ubbo.requestBatteryStatus();
    while (count < 5){
        std::cout << "Battery status: " << my_ubbo.getBatteryStatus() << std::endl;
        const bool* sensor_status = {my_ubbo.getSensorStatus()};
        for (int i = 0; i < 4; i++){
            std::cout << "Sensor status: " << sensor_status[i] << std::endl;
        }
        count++;
        usleep(1000*1000);
    }

}