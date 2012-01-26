#include "minicrusher_base/mc_base.h"
#include "minicrusher_base/mc_structs.h"

namespace minicrusher_base {
  MCBase::MCBase(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io):
    port_(port),
    baud_rate_(baud_rate),
    shutting_down_(false),
    serial_(io, port) {
      serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

  bool MCBase::sendCommandPacket(command_packet_t& packet) {
    return false;
  }

  void MCBase::receivePacket(mc_packet_t& mc_packet) {
    uint8_t temp_char;
    uint8_t start_count(0);
    bool got_valid_packet(false);

    mc_packet_t temp_packet;
    while( !shutting_down_ && !got_valid_packet ) {
      boost::asio::read(serial_, boost::asio::buffer(&temp_char, 1));
      if(start_count == 0) {
        if(temp_char == 'M') {
          start_count = 1;
        }
      }
      else if(start_count == 1) {
        start_count = 0;
        if(temp_char == 'C') {
          //Read an entire packet...
          temp_packet.sync1 = 'M';
          temp_packet.sync2 = 'C';
          boost::asio::read(serial_, boost::asio::buffer(&temp_packet.type, 1));
          boost::asio::read(serial_, boost::asio::buffer(&temp_packet.length, 1));
          boost::asio::read(serial_, boost::asio::buffer(&temp_packet.payload, (temp_packet.length-4)));
        }
      }
    }
  }

}; //namespace minicrusher_base
