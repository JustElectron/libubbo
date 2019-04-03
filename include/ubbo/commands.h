#ifndef _UBBO_COMMANDS_H_
#define _UBBO_COMMANDS_H_

#include <vector>

const uint8_t E_O_F[3] = {0x7F, 0x00, 0x7F};

enum Commands {
    CMD_DRIVE_FORWARD = 0x01,
    CMD_DRIVE_BACKWARD = 0x02,
    CMD_STOP = 0x03,
    CMD_AUTO_DOCK = 0x04,
    CMD_TRANSLATE_LEFT = 0x08,
    CMD_TRANLATE_RIGHT = 0x09,
    CMD_MOVE_TABLET = 0x10,
    CMD_HEARTBEAT = 0x11,
    CMD_VERSION_STATUS = 0x21,
    CMD_BATTERY_STATUS = 0x22
};

std::vector<uint8_t> createPacket(Commands cmd, uint8_t datasize, uint8_t* data);

std::vector<uint8_t> createPacket(Commands cmd, std::vector<uint8_t>* data);

bool verifyPacket(std::vector<uint8_t> packet);

#endif // _UBBO_COMMANDS_H_