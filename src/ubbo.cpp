
#include <iostream>
#include <stdint.h>

#include "serial/serial.h"
#include "ubbo/ubbo.h"
#include "ubbo/commands.h"

namespace ubbo{

size_t Ubbo::printTest(const uint8_t* data, size_t bytes){
    return serial.write(data, bytes);
}

size_t Ubbo::sendPacket(const std::vector<uint8_t>& packet){
    return serial.write(packet);
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
    _serial_timeout = serial::Timeout::simpleTimeout(100);
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
            return true;
        }
    }
    return false;
}

bool Ubbo::isConnected(){
    return serial.isOpen();
}

void Ubbo::disconnect(){
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

void Ubbo::drive(float vel, float angle){

}

void Ubbo::driveForward(uint8_t angle){
    Commands cmd = CMD_DRIVE_FORWARD;
    uint8_t data[] = {0};

    if (angle >= 0 && angle <= 180){
        data[0] = map(angle, 0, 180, 0, 127);
        
        std::vector<uint8_t> packet = createPacket(cmd, 1, data);

        serial.write(packet);
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

        serial.write(packet);
    } else {
        std::cout << "ERROR: angle out of range." << std::endl;
        std::cout << (unsigned)angle << " is not between 0 and 180" << std::endl;
    }
}

void Ubbo::translateLeft(){
    Commands cmd = CMD_TRANSLATE_LEFT;
    std::vector<uint8_t> packet = createPacket(cmd);
    serial.write(packet);
}

void Ubbo::translateRight(){
    Commands cmd = CMD_TRANSLATE_RIGHT;
    std::vector<uint8_t> packet = createPacket(cmd);
    serial.write(packet);
}

void Ubbo::onData(){

}

uint8_t Ubbo::map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

} // end namespace