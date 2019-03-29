#ifndef _UBBO_H_
#define _UBBO_H_

#include "serial/serial.h"

namespace ubbo {
    class Ubbo{
        public:

        Ubbo();

        Ubbo(const std::string& port, const int& baud);

        ~Ubbo();

        bool connect(const std::string& port, const int& baud);

        void disconnect();

        /// @todo add speed to function call
        void driveForward(uint8_t angle);

        /// @todo add speed to function call
        void driveBackward(uint8_t angle);

        /// @todo implement function in arduino
        //void driveWheel(uint8_t vel)

        /// @todo add speed to function call
        void translateLeft();

        /// @todo add speed to function call
        void translateRight();

        private:

        void init();

        void onData();

    }; // end Ubbo class
} // end ubbo namespace

#endif //_UBBO_H_