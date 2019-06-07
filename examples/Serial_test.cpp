#include <iostream>
#include <unistd.h>
#include <chrono>


#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
      Sleep(milliseconds); // 100 ms
#else
      usleep(milliseconds*1000); // 100 ms
#endif
}

void print_vector(std::vector<uint8_t> const& input){
    for (int i = 0; i < input.size(); i++){
        std::cout << (unsigned)input.at(i) << ' ';
    }
}

int main(int argc, char** argv){
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "          Ubbo cpp serial test" << std::endl;
    std::cout << "---------------------------------------" << std::endl;

    // port, baudrate, timeout in milliseconds
    serial::Serial my_serial("/dev/ttyACM0", 57600,
	serial::Timeout::simpleTimeout(250));

    cout << "Is the serial port open?";
    if(my_serial.isOpen())
        cout << " Yes." << endl;
    else
        cout << " No." << endl;

    int count = 0;
    size_t bytes_wrote = 0;
    uint8_t output[10] = {0};
    int corruptBytes = 0;
    int bytes_read = 0;

    usleep(1000*1000*5);
    std::chrono::high_resolution_clock::time_point t1 =
	std::chrono::high_resolution_clock::now();
    while (count < 10000) {

    uint8_t data[10] = {85,85,85,85,85,85,85,85,85,85};
    bytes_wrote += my_serial.write(data, 10);

    bytes_read += my_serial.read(output, 10);

    for (int i = 0; i < sizeof(output); i++){
        if (output[i] != 85){
            std::cout << "Byte number " << count*10 + i <<
		" is corrupt" << std::endl;
            std::cout << "Received " << unsigned(output[i]) << std::endl;
            corruptBytes++;
        }
    }
    usleep(100);
    count++;
    }

    std::chrono::high_resolution_clock::time_point t2 =
        std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>
	( t2 - t1 ).count();

    std::cout << "Time taken: " << duration << std::endl;
    std::cout << "bytes wrote: " << bytes_wrote << std::endl;
    std::cout << "bytes read: " << bytes_read << std::endl;
    std::cout << "corrupt bytes: " << corruptBytes << std::endl;
    std::cout << "done" << std::endl;

    return 0;
}
