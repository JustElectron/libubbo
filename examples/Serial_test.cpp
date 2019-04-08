#include <iostream>
#include <unistd.h>

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
    ubbo::Ubbo my_ubbo("/dev/ttyACM0",9600);

    std::cout << "Serial port is:" << my_ubbo.getPort() << std::endl;

    if (my_ubbo.isConnected()){
        std::cout << "Serial port connected" << std::endl;
    }
    else {
        return 1;
    }

    usleep(5*1000*1000);

    Commands cmd = CMD_TRANSLATE_RIGHT;

    uint8_t data[] = {};

    std::vector<uint8_t> packet = createPacket(cmd, sizeof(data), data);

    print_vector(packet);

    size_t data_send = my_ubbo.sendPacket(packet);
    //usleep(1*1000*1000);
    //size_t data_send2 = my_ubbo.sendPacket(packet);

    std::cout << "data send" << data_send << std::endl;
    /*const uint8_t command[1] = {0x01};
    const uint8_t size[1] = {0x01};
    const uint8_t datas[1] = {0x3C};
    const uint8_t E[1] = {0x7F}; 
    const uint8_t O[1] = {0x00};
    const uint8_t F[1] = {0x7F};

    my_ubbo.printTest(command, 1);
    usleep(100*1000);
    my_ubbo.printTest(size, 1);
    usleep(100*1000);
    my_ubbo.printTest(datas, 1);
    usleep(100*1000);
    my_ubbo.printTest(E, 1);
    usleep(100*1000);
    my_ubbo.printTest(O, 1);
    usleep(100*1000);
    my_ubbo.printTest(F, 1);
    usleep(1*1000*1000);
    my_ubbo.printTest(command, 1);
    usleep(100*1000);
    my_ubbo.printTest(size, 1);
    usleep(100*1000);
    my_ubbo.printTest(datas, 1);
    usleep(100*1000);
    my_ubbo.printTest(E, 1);
    usleep(100*1000);
    my_ubbo.printTest(O, 1);
    usleep(100*1000);
    my_ubbo.printTest(F, 1);
    usleep(100*1000);*/


    std::cout << "done" << std::endl;

    return 0;
}