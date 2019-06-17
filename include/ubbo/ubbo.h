#ifndef _UBBO_H_
#define _UBBO_H_

#include <stdint.h>
#include <chrono>
#include <mutex>
#include <thread>
#include <future>
#include <assert.h>

#include "serial/serial.h"


namespace ubbo {
    class Ubbo{
        public:

        size_t printTest(const uint8_t* data, size_t bytes);

        size_t sendPacket(const std::vector<uint8_t>& packet);

        /**
         * @brief Construct a new empty Ubbo object
         * 
         */
        Ubbo();

        /**
         * @brief Construct a new Ubbo object
         * Robot will try to establish a serial connection on specified port.
         * 
         * @param port The serial port for connection (e.g "/dev/ttyUSB0").
         * @param baud Baud rate for serial connection (defaults to 57600).
         */
        Ubbo(const std::string& port, const uint32_t& baud = 57600);

        /**
         * @brief Destroy the Ubbo object
         * 
         */
        ~Ubbo();

        bool connect();

        /**
         * @brief Try to establish serial connection to the robot.
         * 
         * @param port The serial port for connection (e.g "/dev/ttyUSB0").
         * @param baud Baud rate for serial connection (defaults to 57600).
         * @return true Connection established.
         * @return false Connection failed.
         */
        bool connect(const std::string& port, const uint32_t& baud = 57600);

        /**
         * @brief Check if serial port is open
         * 
         * @return true Serial port is open.
         * @return false Serial port is closed.
         */
        bool isConnected();

        /**
         * @brief Terminate serial connection.
         * 
         */
        void disconnect();

        /**
         * @brief Set the serial port.
         * 
         * @param port The serial port for connection (e.g "/dev/ttyUSB0").
         */
        void setPort(const std::string& port);

        /**
         * @brief Set the serial baud rate.
         * 
         * @param baud Baud rate for serial connection (defaults to 57600).
         */
        void setBaud(const uint32_t& baud);

        /**
         * @brief Get the serial port name.
         * 
         * @return std::string Serial port name.
         */
        std::string getPort();

        /**
         * @brief Get the serial baud rate.
         * 
         * @return std::string Serial baud rate.
         */
        uint32_t getBaud();

        /**
         * @brief Drive robot.
         * 
         * @param vel Set target velocity for the robot.
         * @param angle Set angular velocity for the robot.
         * 
         * @todo Configure vel for [x,y].
         */
        size_t drive(float vel_x, float vel_y, float ang_z);

        void stop();

        /**
         * @brief Drive robot forward at a target angle.
         * 
         * @param angle Target angle for the robot to turn. Possible range 0-180.
         * 
         * @todo add speed to function call.
         */
        void driveForward(uint8_t angle);

        /**
         * @brief Drive robot backward at a target angle.
         * 
         * @param angle Target angle for the robot to turn. Possible range 0-180.
         * 
         * @todo add speed to function call
         */
        void driveBackward(uint8_t angle);

        /// @todo implement function in arduino
        //void driveWheel(uint8_t vel)

        /**
         * @brief Translate left.
         * 
         * @todo add speed to function call
         */
        void translateLeft();

        /**
         * @brief Translate right.
         * 
         * @todo add speed to function call
         */
        void translateRight();

        void requestBatteryStatus();

        /**
         * @brief Move tablet mount.
         * 
         * @param angle Move tablet. Possible range 0-180.
         */
        void moveTablet(uint8_t angle);

        size_t available();

        std::string read(size_t size=1);

        std::vector<uint8_t> readBuffer();

        int getBatteryStatus();

        const uint8_t* getVersionStatus();

        uint8_t getSensorStatus();

        bool getProxFront();

        bool getProxRight();
        
        bool getProxBack();
        
        bool getProxLeft();

        //long getEncoderPosition();

        struct position {
            position(): x(0.0), y(0.0), z(0.0),
                yaw(0.0), pitch(0.0), roll(0.0) {}

            double x;
            double y;
            double z;

            double yaw;
            double pitch;
            double roll;
        } position;

        struct twist {
            twist(): linear_x(0.0), linear_y(0.0), linear_z(0.0),
                angular_x(0.0), angular_y(0.0), angular_z(0.0) {}

            float linear_x;
            float linear_y;
            float linear_z;

            float angular_x;
            float angular_y;
            float angular_z;
        } twist;

        

        private:

        /**
         * @brief Initialize robot parameters.
         * 
         */
        void init();

        /**
         * @brief Event handler for incoming serial data.
         * 
         */
        void onData(std::future<void> futureObj);

        void startReading();

        void stopReading();

        bool verifyPacket(std::vector<uint8_t> packet, std::vector<uint8_t>::iterator it);

        uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);

        uint8_t mapFloatToUInt(float x, float in_min, float in_max, uint8_t out_min, uint8_t out_max);

        std::string _port;
        uint32_t _baud;
        int _connection_retry;

        int _wheel_circ_mm;
        float _wheel_dist_x;
        float _wheel_dist_y;

        uint8_t _battery_status;
        uint8_t _version_status[2];

        bool _wall_sensor_front;
        bool _wall_sensor_right;
        bool _wall_sensor_back;
        bool _wall_sensor_left;

        double _front_right_dist;
        double _back_right_dist;
        double _back_left_dist;
        double _front_left_dist;

        float _front_right_vel;
        float _back_right_vel;
        float _back_left_vel;
        float _front_left_vel;

        int _PPR;

        bool fr, br, bl, fl;

        serial::Serial serial;
        serial::Timeout _serial_timeout;

        std::chrono::high_resolution_clock::time_point _last_send;
        std::vector<uint8_t> _last_packet;

        std::thread _serial_handler;
        std::vector<uint8_t> _buffer;
        std::promise<void> _serial_exit_signal;
        std::future<void> _futureObj;

        std::mutex _serial_mutex;

    }; // end Ubbo class
} // end ubbo namespace

#endif //_UBBO_H_