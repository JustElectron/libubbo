
#include <iostream>
#include <stdint.h>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <mutex>
#include <math.h>
#include <future>
#include <assert.h>
#include <algorithm>

#include "serial/serial.h"
#include "ubbo/ubbo.h"
#include "ubbo/commands.h"

void print_vectors(std::vector<uint8_t> const& input){
    std::cout << "printing" << std::endl;
    for (int i = 0; i < input.size(); i++){
        std::cout << (unsigned)input.at(i) << ' ';
    }
}

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
    _serial_timeout = serial::Timeout::simpleTimeout(100);
    _battery_status = 0;
    _version_status[0] = 0;
    _version_status[1] = 0;

    _wall_sensor_front = false;
    _wall_sensor_right = false;
    _wall_sensor_back = false;
    _wall_sensor_left = false;

    _front_right_dist = 0.0;
    _back_right_dist = 0.0;
    _back_left_dist = 0.0;
    _front_left_dist = 0.0;

    _front_right_vel = 0.0;
    _back_right_vel = 0.0;
    _back_left_vel = 0.0;
    _front_left_vel = 0.0;
    _wheel_circ_mm = 300.0;
    _wheel_dist_x = 300.0;
    _wheel_dist_y = 300.0;
    _PPR = 12 * 64;
    _buffer.erase(_buffer.begin(), _buffer.end());
    fr = false;
    br = false;
    bl = false;
    fl = false;

    
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
            usleep(3*1000*1000);
            std::string flush = serial.read(serial.available());
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
            usleep(3*1000*1000);
            std::string flush = serial.read(serial.available());
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
    serial.setBaudrate(baud);
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

void Ubbo::stop(){
    Commands cmd = CMD_STOP;

    std::vector<uint8_t> packet = createPacket(cmd);
    // Send packet
    sendPacket(packet);
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
    // Set command
    Commands cmd = CMD_BATTERY_STATUS;
    // Create packet
    std::vector<uint8_t> packet = createPacket(cmd);
    // Send packet
    sendPacket(packet);
}

size_t Ubbo::available(){
    // Return bytes available in the serial buffer
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
    // Return battery status
    return _battery_status;
}

const uint8_t* Ubbo::getVersionStatus(){
    // Return version status reference
    return _version_status;
}

uint8_t Ubbo::getSensorStatus(){
    uint8_t sensor_status = 0;
    // Return sensor status reference
    if (_wall_sensor_front) sensor_status += 1;
    if (_wall_sensor_right) sensor_status += 2;
    if (_wall_sensor_back) sensor_status += 4;
    if (_wall_sensor_left) sensor_status += 8;
    return sensor_status;
}

bool Ubbo::getProxFront(){
    return _wall_sensor_front;
}

bool Ubbo::getProxRight(){
    return _wall_sensor_right;
}

bool Ubbo::getProxBack(){
    return _wall_sensor_back;
}

bool Ubbo::getProxLeft(){
    return _wall_sensor_left;
}

void Ubbo::onData(std::future<void> futureObj){
    // Run while promise is not set
    while (futureObj.wait_for(std::chrono::microseconds(10))
            == std::future_status::timeout)
	{
        // Get number of available bytes
        size_t bytes_available = serial.available();
		if (bytes_available){
            // Read number of available bytes into buffer
            serial.read(_buffer, bytes_available);
            // https://stackoverflow.com/questions/10508392/looking-to-find-a-c-stl-vector-inside-an-stl-vector
            // Search buffer for EOF
            while (true){
                auto it = std::search(_buffer.begin(),_buffer.end(),
                E_O_F_Vec.begin(),E_O_F_Vec.end());
                if(it != _buffer.end()){
                    // Verify that buffer contains a package
                    if (verifyPacket(_buffer, it)){
                        // Erase buffer from beginning to EOF
                        _buffer.erase(_buffer.begin(), it + E_O_F_Vec.size());
                    }
                    else {
                        auto it2 = std::search(it+3,_buffer.end(),
                            E_O_F_Vec.begin(),E_O_F_Vec.end());
                        if (verifyPacket(_buffer, it2)){
                            // Erase buffer from beginning to EOF
                            _buffer.erase(_buffer.begin(), it + E_O_F_Vec.size());
                        }
                    }
                }
                else{
                    break;
                }
            }
        }
        // Sleep thread for 10 ms
		std::this_thread::sleep_for(std::chrono::microseconds(10));

	}
}

void Ubbo::startReading(){
    // Set future object with exit signal promise
    _futureObj = _serial_exit_signal.get_future();
    // Start thread for reading data
    _serial_handler = std::thread(&Ubbo::onData, this, std::move(_futureObj));
}

void Ubbo::stopReading(){
    // Set exit signal for readign thread
    _serial_exit_signal.set_value();
    // Wait for thread to join
    _serial_handler.join();
    // Clear promise 
    _serial_exit_signal = std::promise<void>();
}

bool Ubbo::verifyPacket(std::vector<uint8_t> packet, std::vector<uint8_t>::iterator it){
    // Minimum data package size
    if (packet.size() < 5){
        return false;
    }

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
                bool sensor_status = false;
                if (packet[index - 1] == 1){
                    sensor_status = true;
                }
                if (packet[index - 2] == 0) _wall_sensor_front = sensor_status;
                if (packet[index - 2] == 1) _wall_sensor_right = sensor_status;
                if (packet[index - 2] == 2) _wall_sensor_back = sensor_status;
                if (packet[index - 2] == 3) _wall_sensor_left = sensor_status;
            }
            else{
                return false;
            }
        }
        else if (packet[index - 5] == CMD_ENCODER_STATUS){
            if (packet[index - 4] == 3){
                
                switch (packet[index - 3]){
                    case 1:
                    {
                        int curr_pulse = (short)(packet[index - 2] << 8 | packet[index -1]);
                        _front_right_dist += (double)curr_pulse/_PPR*(_wheel_circ_mm-3);
                        fr = true;
                        break;
                    }
                    
                    case 2:
                    {
                        int curr_pulse = (short)(packet[index - 2] << 8 | packet[index -1]);
                        _back_right_dist += (double)curr_pulse/_PPR*(_wheel_circ_mm-3);
                        br = true;
                        break;
                    }
                    
                    case 3:
                    {
                        int curr_pulse = (short)(packet[index - 2] << 8 | packet[index -1]);
                        _back_left_dist += (double)curr_pulse/_PPR*_wheel_circ_mm;
                        bl = true;
                        break;
                    }

                    case 4:
                    {
                        int curr_pulse = (short)(packet[index - 2] << 8 | packet[index -1]);
                        _front_left_dist += (double)curr_pulse/_PPR*_wheel_circ_mm;
                        fl = true;
                        break;
                    }
                    
                    default:
                        break;
                }
                if (fr && br && bl && fl){
                    double dist_x = (_front_right_dist + _back_right_dist
                        + _back_left_dist + _front_left_dist) / 4;
                    double dist_y = (_front_right_dist - _back_right_dist
                        + _back_left_dist - _front_left_dist) / 4;
                    double rot_z = (_front_right_dist + _back_right_dist
                        - _back_left_dist - _front_left_dist) / 4;

                    double delta_x = dist_x*cos(position.yaw) - dist_y*sin(position.yaw);
                    double delta_y = dist_x*sin(position.yaw) + dist_y*cos(position.yaw);
                    double delta_z = rot_z / ((_wheel_dist_x / 2) + (_wheel_dist_y) / 2);

                    delta_x = delta_x + (0.0185 * delta_x * fabs(twist.linear_x)/0.4);
                    delta_y = delta_y + (-0.134 * delta_y * fabs(twist.linear_y)/0.4);
                    delta_z = delta_z + (0.033 * delta_z * fabs(twist.angular_z)/1);

                    position.x += delta_x/1000;
                    position.y += delta_y/1000;
                    position.yaw += delta_z;

                    _front_left_dist = 0;
                    _front_right_dist = 0;
                    _back_left_dist = 0;
                    _back_right_dist = 0;

                    fr = false;
                    br = false;
                    bl = false;
                    fl = false;
                }
            }
            else {
                return false;
            }
        }
        else if (packet[index - 5] == CMD_VELOCITY_STATUS){
            if (packet[index - 4] == 3){
                switch (packet[index - 3]){
                    case 1:
                    {
                        int curr_vel = (short)(packet[index - 2] << 8 | packet[index -1]);
                        _front_right_vel = (float)curr_vel/1000; // mm/s -> m/s 
                        fr = true;
                        break;
                    }
                    
                    case 2:
                    {
                        int curr_vel = (short)(packet[index - 2] << 8 | packet[index -1]);
                        _back_right_vel = (float)curr_vel/1000; // mm/s -> m/s 
                        br = true;
                        break;
                    }
                    
                    case 3:
                    {
                        int curr_vel = (short)(packet[index - 2] << 8 | packet[index -1]);
                        _back_left_vel = (float)curr_vel/1000; // mm/s -> m/s 
                        bl = true;
                        break;
                    }

                    case 4:
                    {
                        int curr_vel = (short)(packet[index - 2] << 8 | packet[index -1]);
                        _front_left_vel = (float)curr_vel/1000; // mm/s -> m/s 
                        fl = true;
                        break;
                    }
                    
                    default:
                        break;
                }
                if (fr && br && bl && fl){
                    float lin_vel_x = (_front_right_vel + _back_right_vel
                        + _back_left_vel + _front_left_vel) / 4;
                    float lin_vel_y = (_front_right_vel - _back_right_vel
                        + _back_left_vel - _front_left_vel) / 4;
                    float ang_vel_z = (_front_right_vel + _back_right_vel
                        - _back_left_vel - _front_left_vel) / 4;

                    float ang_rad_z = ang_vel_z / 
                        ((_wheel_dist_x / 2) + (_wheel_dist_y / 2));

                    twist.linear_x = lin_vel_x + (0.013 * lin_vel_x);
                    twist.linear_y = lin_vel_y + (-0.143 * lin_vel_y);
                    twist.angular_z = ang_rad_z*1000 + (0.015 * ang_rad_z);

                    fr = false;
                    br = false;
                    bl = false;
                    fl = false;
                }
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


