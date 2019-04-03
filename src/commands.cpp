#include <vector>

#include "ubbo/commands.h"

std::vector<uint8_t> createPacket(Commands* cmd, uint8_t data_size, uint8_t* data){
    std::vector<uint8_t> data_packet{(uint8_t)cmd, data_size};
    data_packet.insert(data_packet.begin(), std::begin(data), std::end(data));
    data_packet.push_back(EOF);
}

std::vector<uint8_t> createPacket(Commands cmd, std::vector<uint8_t>* data){
    
}

bool verifyPacket(std::vector<uint8_t> packet){

}