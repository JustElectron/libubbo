#include <iostream>
#include <unistd.h>
#include <chrono>
#include <math.h>

#include "ubbo/ubbo.h"
#include "ubbo/commands.h"

void print_vector(std::vector<uint8_t> const& input){
    std::cout << "printing" << std::endl;
    for (int i = 0; i < input.size(); i++){
        std::cout << (unsigned)input.at(i) << ' ';
    }
}

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

    std::vector<uint8_t> buffer;
    for (int i = 0; i < 10; i++){
        buffer = my_ubbo.readBuffer();
        print_vector(buffer);
        usleep(1000*1000);
    }

    return 0;
}