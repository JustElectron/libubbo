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
    ubbo::Ubbo* my_ubbo = new ubbo::Ubbo("/dev/ttyACM0",57600);

    std::cout << "Serial port is:" << my_ubbo->getPort() << std::endl;

    if (my_ubbo->isConnected()){
        std::cout << "Serial port connected" << std::endl;
    }
    else {
        return 1;
    }

    usleep(5*1000*1000);

    Commands cmd = CMD_TRANSLATE_RIGHT;

    uint8_t data[] = {};

    std::vector<uint8_t> packet1 = createPacket(cmd, sizeof(data), data);
    Commands cmd2 = CMD_DRIVE_FORWARD;
    uint8_t data2[] = {1};
    std::vector<uint8_t> packet2 = createPacket(cmd2, sizeof(data2), data2);

    
    std::chrono::high_resolution_clock::time_point t[100];
    size_t data_send[99];
    int count = 0;
    while (count < 100){
        //std::cout << "This threads" << std::this_thread::get_id() << std::endl;
        if (count == 9){
            t[count] = std::chrono::high_resolution_clock::now();
            data_send[count] = my_ubbo->sendPacket(packet2);
        }
        else{
            t[count] = std::chrono::high_resolution_clock::now();
            data_send[count] = my_ubbo->sendPacket(packet1);
        }
        count++;
    }
    t[10] = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 100; i++){
        std::cout << "data send" << data_send[i] << std::endl;
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t[i+1]-t[i]).count();
        std::cout << "duration" << duration << std::endl;
    }

    std::cout << "done" << std::endl;

    return 0;
}