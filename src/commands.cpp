#include <vector>
#include <stdint.h>

#include "ubbo/commands.h"

std::vector<uint8_t> createPacket(const Commands& cmd, uint8_t data_size, uint8_t* data){
    std::vector<uint8_t> data_packet;
    data_packet.push_back((uint8_t)cmd);
    data_packet.push_back(data_size);
    for (int i = 0; i < data_size; i++){
        data_packet.push_back(data[i]);
    }
    data_packet.push_back(E_O_F[0]);
    data_packet.push_back(E_O_F[1]);
    data_packet.push_back(E_O_F[2]);

    return data_packet;
}

std::vector<uint8_t> createPacket(const Commands& cmd, const std::vector<uint8_t>& data){
    
}

std::vector<uint8_t> createPacket(const Commands& cmd){
    std::vector<uint8_t> data_packet;
    data_packet.push_back((uint8_t)cmd);
    data_packet.push_back(0x00);
    data_packet.push_back(E_O_F[0]);
    data_packet.push_back(E_O_F[1]);
    data_packet.push_back(E_O_F[2]);

    return data_packet;
}

bool verifyPacket(std::vector<uint8_t> packet){
    return true;
}