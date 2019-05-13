
#include <iostream>
#include <stdint.h>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <mutex>
#include <future>
#include <assert.h>
#include <algorithm>

#include "serial/serial.h"
#include "ubbo/ubbo.h"
#include "ubbo/commands.h"

namespace ubbo{

size_t Ubbo::printTest(const uint8_t* data, size_t bytes){
    return serial.write(data, bytes);
}

size_t Ubbo::sendPacket(const std::vector<uint8_t>& packet){
    _serial_mutex.lock();
    //std::cout << "This thread" << std::this_thread::get_id() << std::endl;
    // Get time now;
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
    // now in microseconds
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>
                    (now - _last_send).count();
    // Only send if more than 100 us since last send
    // anti flood of serial port
    if (now_us >= 1000){
        // Send data
        _last_send = now;
        _last_packet = packet;
        size_t bytes_wrote = serial.write(packet);
        //std::cout << "Sending packet" << std::endl;
        _serial_mutex.unlock();
        return bytes_wrote;
    }
    else{
        //std::cout << "last packet: " << unsigned(_last_packet[0]) << " current packet: " << unsigned(packet[0]) << std::endl;
        // Return -1 if no significant change in packet
        if (_last_packet[0] == packet[0]
            && abs(_last_packet[2]-packet[2]) <= 5) {
            //std::cout << "package discarded" << std::endl;
            _serial_mutex.unlock();
            return 100;
        }
        // special case for drive forward and drive backward conatining 3 variables
        else if((_last_packet[0] == packet[0] == 0x40
                || _last_packet[0] == packet[0] == 0x41)
                && (abs(_last_packet[2]-packet[2]) <= 5
                || abs(_last_packet[3]-packet[3]) <= 5
                || abs(_last_packet[4]-packet[4]) <= 5)){
                _serial_mutex.unlock();
                return 100;
        }
        else{
            // sleep until last_send + 100 and try again
            //std::cout << "waiting" << std::endl;
            auto sleep_duration_us = 100 - now_us;
            usleep(abs(sleep_duration_us));
            _serial_mutex.unlock();
            return sendPacket(packet);
        }
    }
}

Ubbo::Ubbo(){
    
    init();
}

Ubbo::Ubbo(const std::string& port, const uint32_t& baud){
    
    init();
    connect(port, baud);
}

Ubbo::~Ubbo(){
    disconnect();
}

void Ubbo::init(){
    _port = "";
    _baud = 57600;
    _connection_retry = 5;
    _serial_timeout = serial::Timeout::simpleTimeout(1000);
    
}

bool Ubbo::connect(){
    serial.setPort(_port);
    serial.setBaudrate(_baud);
    serial.setTimeout(_serial_timeout);

    for (int i=0; i<_connection_retry; i++){
        if (!isConnected()){
            serial.open();
        }
        else{
            startReading();
            return true;
        }
    }
    return false;
}

bool Ubbo::connect(const std::string& port, const uint32_t& baud){
    _port = port;
    _baud = baud;
    serial.setPort(port);
    serial.setBaudrate(baud);
    serial.setTimeout(_serial_timeout);

    for (int i=0; i<_connection_retry; i++){
        if (!isConnected()){
            serial.open();
        }
        else{
            startReading();
            return true;
        }
    }
    return false;
}

bool Ubbo::isConnected(){
    return serial.isOpen();
}

void Ubbo::disconnect(){
    stopReading();
    serial.close();
}

void Ubbo::setPort(const std::string& port){
    _port = port;

}

void Ubbo::setBaud(const uint32_t& baud){
    _baud = baud;
}

std::string Ubbo::getPort(){
    return serial.getPort();
}

uint32_t Ubbo::getBaud(){
    return serial.getBaudrate();
}

size_t Ubbo::drive(float vel_x, float vel_y, float ang_z){
    Commands cmd = CMD_VEL_FORWARD;
    uint8_t data[] = {0};

    if (vel_x > 0.4){vel_x = 0.4;}
    else if (vel_x < -0.4){vel_x = -0.4;}
    if (vel_y > 0.45){vel_y = 0.45;}
    else if (vel_y < -0.45){vel_y = -0.45;}
    if (ang_z > 1.0){ang_z = 1.0;}
    else if (ang_z < -1.0){ang_z = -1.0;}
    uint8_t x = mapFloatToUInt(vel_x, -0.4, 0.4, 0, 255);
    uint8_t y = mapFloatToUInt(vel_y, -0.45, 0.45, 0, 255);
    uint8_t z = mapFloatToUInt(ang_z, -1, 1, 0, 255);

    data[0] = x; data[1] = y; data[2] = z;

    std::vector<uint8_t> packet = createPacket(cmd, 3, data);
    return sendPacket(packet);
}

void Ubbo::driveForward(uint8_t angle){
    Commands cmd = CMD_DRIVE_FORWARD;
    uint8_t data[] = {0};

    if (angle >= 0 && angle <= 180){
        data[0] = map(angle, 0, 180, 0, 127);
        
        std::vector<uint8_t> packet = createPacket(cmd, 1, data);

        sendPacket(packet);
    } else {
        std::cout << "ERROR: angle out of range." << std::endl;
        std::cout << (unsigned)angle << " is not between 0 and 180" << std::endl;
    }
}

void Ubbo::driveBackward(uint8_t angle){
    Commands cmd = CMD_DRIVE_BACKWARD;
    uint8_t data[] = {0};

    if (angle >= 0 && angle <= 180){
        data[0] = map(angle, 0, 180, 0, 127);
        
        std::vector<uint8_t> packet = createPacket(cmd, 1, data);

        sendPacket(packet);
    } else {
        std::cout << "ERROR: angle out of range." << std::endl;
        std::cout << (unsigned)angle << " is not between 0 and 180" << std::endl;
    }
}

void Ubbo::translateLeft(){
    Commands cmd = CMD_TRANSLATE_LEFT;
    std::vector<uint8_t> packet = createPacket(cmd);
    sendPacket(packet);
}

void Ubbo::translateRight(){
    Commands cmd = CMD_TRANSLATE_RIGHT;
    std::vector<uint8_t> packet = createPacket(cmd);
    sendPacket(packet);
}

size_t Ubbo::available(){
    return serial.available();
}

std::string Ubbo::read(size_t size){
    return serial.read(size);
}

std::vector<uint8_t> Ubbo::readBuffer(){
    std::vector<uint8_t> x = _buffer;
    //_buffer.clear();
    return x;
}

void Ubbo::onData(std::future<void> futureObj){
    while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
	{
        size_t bytes_available = serial.available();
		if (bytes_available){
            serial.read(_buffer, bytes_available);
            std::cout << "Reading: " << bytes_available << std::endl;
            // https://stackoverflow.com/questions/10508392/looking-to-find-a-c-stl-vector-inside-an-stl-vector
            auto it = std::search(_buffer.begin(),_buffer.end(),
                E_O_F_Vec.begin(),E_O_F_Vec.end());
            if(it != _buffer.end()){
                std::cout << "EOF found at offset "
                            << it - _buffer.begin() << '\n';
                if (verifyPacket(_buffer)){
                    _buffer.erase(_buffer.begin(), it + E_O_F_Vec.size());
                }
            }
            else{
                std::cout << "EOF not found\n";
            }
        }
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

	}
}

void Ubbo::startReading(){
    _futureObj = _serial_exit_signal.get_future();
    _serial_handler = std::thread(&Ubbo::onData, this, std::move(_futureObj));
}

void Ubbo::stopReading(){
    _serial_exit_signal.set_value();
    _serial_handler.join();
    // Clear promise 
    _serial_exit_signal = std::promise<void>();
    std::cout << "Thread stopped" << std::endl;
}

uint8_t Ubbo::map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t Ubbo::mapFloatToUInt(float x, float in_min, float in_max, uint8_t out_min, uint8_t out_max){
    return (uint8_t)((x - in_min) * (out_max-out_min) / (in_max - in_min) + out_min);
}

} // end namespace