
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

size_t Ubbo::sendPacket(const std::vector<uint8_t>& packet){
    _serial_mutex.lock();
    // Get time now;
    std::chrono::high_resolution_clock::time_point now =
        std::chrono::high_resolution_clock::now();
    // Get time since last send
    auto timeout_us = std::chrono::duration_cast<std::chrono::microseconds>
        (now - _last_send).count();
    // Only send if more than 10 ms since last send
    // anti flood of serial port
    if (timeout_us >= 10*1000){
        // Update last send 
        _last_send = now;
        // Update last package
        _last_packet = packet;
        // Send packet
        size_t bytes_wrote = serial.write(packet);
        _serial_mutex.unlock();
        // Return number of bytes wrote
        return bytes_wrote;
    }
    else{
        // Return 100 if no significant change in packet
        // instead of -1 due to unsigned return
        // @TODO Consider changeing to int using static_cast
        // https://stackoverflow.com/questions/22184403/how-to-cast-the-size-t-to-double-or-int-c
        if (_last_packet[0] == packet[0]
            && abs(_last_packet[2]-packet[2]) <= 5) {
            _serial_mutex.unlock();
            return 100;
        }
        // special case for drive forward and drive backward
        // conatining 3 variables
        else if((_last_packet[0] == packet[0] == 0x40
                || _last_packet[0] == packet[0] == 0x41)
                && (abs(_last_packet[2]-packet[2]) <= 5
                || abs(_last_packet[3]-packet[3]) <= 5
                || abs(_last_packet[4]-packet[4]) <= 5)){
                _serial_mutex.unlock();
                return 100;
        }
        else{
            // sleep until last_send + 10 ms and try again
            auto sleep_duration_us = 10*1000 - timeout_us;
            usleep(abs(sleep_duration_us));
            _serial_mutex.unlock();
            return sendPacket(packet);
        }
    }
}

Ubbo::Ubbo(){
    // Initialize variables
    init();
}

Ubbo::Ubbo(const std::string& port, const uint32_t& baud){
    // initialize variables and connect to serial port
    init();
    if (!connect(port, baud)){
        // @TODO Change to throw
        std::cout << "Could not connect to serial port";
        std::cout << port << std::endl;
    }
}

Ubbo::~Ubbo(){
    disconnect();
}

void Ubbo::init(){
    // Initialize member variables
    _port = "";
    _baud = 57600;
    _connection_retry = 5;
    _serial_timeout = serial::Timeout::simpleTimeout(1000);
    _battery_status = 0;
    _version_status[0] = 0;
    _version_status[1] = 0;
    for (int i = 0; i < 4; i++){
        _sensor_status[i] = false;
    }
    
}

bool Ubbo::connect(){
    // Setup Serial port connection
    serial.setPort(_port);
    serial.setBaudrate(_baud);
    serial.setTimeout(_serial_timeout);

    // try connecting
    for (int i=0; i<_connection_retry; i++){
        if (!isConnected()){
            serial.open();
        }
        else{
            // Start sensor readings
            startReading();
            return true;
        }
    }
    return false;
}

bool Ubbo::connect(const std::string& port, const uint32_t& baud){
    // Setup Serial port connection
    _port = port;
    _baud = baud;
    serial.setPort(port);
    serial.setBaudrate(baud);
    serial.setTimeout(_serial_timeout);

    // try connecting
    for (int i=0; i<_connection_retry; i++){
        if (!isConnected()){
            serial.open();
        }
        else{
            // Start sensor readings
            startReading();
            return true;
        }
    }
    return false;
}

bool Ubbo::isConnected(){
    // Check if serial port is connected
    return serial.isOpen();
}

void Ubbo::disconnect(){
    // Stop sensor reading
    stopReading();
    // Close serial port
    serial.close();
}

void Ubbo::setPort(const std::string& port){
    // Set port member var and serial object
    _port = port;
    serial.setPort(port);
}

void Ubbo::setBaud(const uint32_t& baud){
    // Set baud member var and serial object
    _baud = baud;
    serial.setBaud(baud);
}

std::string Ubbo::getPort(){
    // Return serial port
    return serial.getPort();
}

uint32_t Ubbo::getBaud(){
    // Return serial baud rate
    return serial.getBaudrate();
}

size_t Ubbo::drive(float vel_x, float vel_y, float ang_z){
    // Set command to be send
    Commands cmd = CMD_VEL_FORWARD;
    // Initialize data array
    uint8_t data[] = {0};

    // limit velocities:
    // vel x ranges from:  -0.4m/s to 0.4m/s
    if (vel_x > 0.4){vel_x = 0.4;}
    else if (vel_x < -0.4){vel_x = -0.4;}
    // vel y ranges from:   -0.45m/s to 0.45m/s
    if (vel_y > 0.45){vel_y = 0.45;}
    else if (vel_y < -0.45){vel_y = -0.45;}
    // ang z ranges from:   -1rad/s to 1rad/s
    if (ang_z > 1.0){ang_z = 1.0;}
    else if (ang_z < -1.0){ang_z = -1.0;}

    // map floating point to byte
    uint8_t x = mapFloatToUInt(vel_x, -0.4, 0.4, 0, 255);
    uint8_t y = mapFloatToUInt(vel_y, -0.45, 0.45, 0, 255);
    uint8_t z = mapFloatToUInt(ang_z, -1, 1, 0, 255);

    // Insert velocities to data array
    data[0] = x; data[1] = y; data[2] = z;

    // Create packet
    std::vector<uint8_t> packet = createPacket(cmd, 3, data);
    return sendPacket(packet);
}

void Ubbo::driveForward(uint8_t angle){
    // Set Command
    Commands cmd = CMD_DRIVE_FORWARD;
    // initialize data array
    uint8_t data[] = {0};

    // Check if angle is within range
    if (angle >= 0 && angle <= 180){
        // Map angle to a half byte
        data[0] = map(angle, 0, 180, 0, 127);
        
        // Create packet
        std::vector<uint8_t> packet = createPacket(cmd, 1, data);
        // Send packet
        sendPacket(packet);
    } else {
        // Display error message if out of range
        std::cout << "ERROR: angle out of range." << std::endl;
        std::cout << (unsigned)angle;
        std::cout << " is not between 0 and 180" << std::endl;
    }
}

void Ubbo::driveBackward(uint8_t angle){
    // Set Command
    Commands cmd = CMD_DRIVE_BACKWARD;
    // initialize data array
    uint8_t data[] = {0};

    // Check if angle is within range
    if (angle >= 0 && angle <= 180){
        // Map angle to a half byte
        data[0] = map(angle, 0, 180, 0, 127);
        
        // Create packet
        std::vector<uint8_t> packet = createPacket(cmd, 1, data);
        // Send packet
        sendPacket(packet);
    } else {
        // Display error message if out of range
        std::cout << "ERROR: angle out of range." << std::endl;
        std::cout << (unsigned)angle;
        std::cout << " is not between 0 and 180" << std::endl;
    }
}

void Ubbo::translateLeft(){
    // Set command
    Commands cmd = CMD_TRANSLATE_LEFT;
    // Create package
    std::vector<uint8_t> packet = createPacket(cmd);
    // Send packet
    sendPacket(packet);
}

void Ubbo::translateRight(){
    // Set command
    Commands cmd = CMD_TRANSLATE_RIGHT;
    // Create packet
    std::vector<uint8_t> packet = createPacket(cmd);
    // Send packet
    sendPacket(packet);
}

void Ubbo::requestBatteryStatus(){
    Commands cmd = CMD_BATTERY_STATUS;
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

int Ubbo::getBatteryStatus(){
    return _battery_status;
}

const uint8_t* Ubbo::getVersionStatus(){
    return _version_status;
}

const bool* Ubbo::getSensorStatus(){
    return _sensor_status;
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
                //std::cout << "EOF found at offset "
                //            << it - _buffer.begin() << '\n';
                if (verifyPacket(_buffer)){
                    _buffer.erase(_buffer.begin(), it + E_O_F_Vec.size());
                }
            }
            else{
                //std::cout << "EOF not found\n";
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
}

bool Ubbo::verifyPacket(std::vector<uint8_t> packet){
    // Minimum data package size
    if (packet.size() < 5){
        return false;
    }
    // Search for EOF and create iterator at start of it
    auto it = std::search(_buffer.begin(),_buffer.end(),
                E_O_F_Vec.begin(),E_O_F_Vec.end());
    // Validate iterator
    if(it != _buffer.end()){
        // Convert iterator to index
        int index = std::distance(packet.begin(), it);
        if (packet[index - 3] == CMD_BATTERY_STATUS){
            // Battery package received
            // Validate package size
            if (packet[index - 2] == 1){
                _battery_status = packet[index - 1];
            }
            else{
                return false;
            }
        }
        else if (packet[index - 4] == CMD_VERSION_STATUS){
            // Version status received
            // Validate package size
            if (packet[index -3] == 2){
                _version_status[0] = packet[index - 2];
                _version_status[1] = packet[index - 1];
            }
            else{
                return false;
            }
        }
        else if (packet[index - 4] == CMD_SENSOR_STATUS){
            // Sensor status received
            // Validate package size
            if (packet[index - 3] == 2){
                if (packet[index - 1] == 1){
                    _sensor_status[packet[index - 2]] = true;
                }
                else {
                    _sensor_status[packet[index - 2]] = false;
                }
            }
            else{
                return false;
            }
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
    return true;
}

uint8_t Ubbo::map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

uint8_t Ubbo::mapFloatToUInt(float x, float in_min, float in_max, uint8_t out_min, uint8_t out_max){
    return (uint8_t)((x - in_min) * (out_max-out_min) / (in_max - in_min) + out_min);
}

} // end namespace