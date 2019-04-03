
#include <iostream>

#include "serial/serial.h"
#include "ubbo/ubbo.h"
#include "ubbo/commands.h"

namespace ubbo{

size_t Ubbo::printTest(const uint8_t* data, size_t bytes){
    return serial.write(data, bytes);
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

}

void Ubbo::driveBackward(uint8_t angle){

}

void Ubbo::translateLeft(){

}

void Ubbo::translateRight(){

}

void Ubbo::onData(){

}

} // end namespace