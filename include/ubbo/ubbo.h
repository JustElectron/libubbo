#ifndef _UBBO_H_
#define _UBBO_H_

#include "serial/serial.h"

namespace ubbo {
    class Ubbo{
        public:

        size_t printTest(const uint8_t* data, size_t bytes);

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
        void drive(float vel, float angle);

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

        /**
         * @brief Move tablet mount.
         * 
         * @param angle Move tablet. Possible range 0-180.
         */
        void moveTablet(uint8_t angle);

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
        void onData();

        std::string _port;
        uint32_t _baud;
        int _connection_retry;

        serial::Serial serial;
        serial::Timeout _serial_timeout;

    }; // end Ubbo class
} // end ubbo namespace

#endif //_UBBO_H_