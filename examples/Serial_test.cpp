#include <iostream>
#include <unistd.h>

#include "ubbo/ubbo.h"


int main(int argc, char** argv){
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "          Ubbo cpp serial test" << std::endl;
    std::cout << "---------------------------------------" << std::endl;
    ubbo::Ubbo my_ubbo("/dev/ttyACM0");

    std::cout << "Serial port is:" << my_ubbo.getPort() << std::endl;

    if (my_ubbo.isConnected()){
        std::cout << "Serial port connected" << std::endl;
    }
    else {
        return 1;
    }

    uint8_t data[] = {1,2,3,4,5,6,7,8,9}; 
    int n = 0;
    while (n < 10){
        size_t printed = my_ubbo.printTest(data, sizeof(data));
        std::cout << printed << " bytes send" << std::endl;
        n++;
        usleep(1000*1000);
    }

    std::cout << "done" << std::endl;

    return 0;
}