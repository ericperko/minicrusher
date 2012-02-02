#include "minicrusher_base/mc_base.h"
#include "minicrusher_base/mc_structs.h"
#include <endian.h>
#include <boost/crc.hpp>

namespace minicrusher_base {
  typedef boost::crc_optimal<16, 0x1021, 0xFFFF, 0xFFFF, true, true> mc_crc_t;
  static const uint16_t good_checksum = 0xF0B8;

  MCBase::MCBase(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io):
    port_(port),
    baud_rate_(baud_rate),
    shutting_down_(false),
    serial_(io, port) {
      serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }

  bool MCBase::sendCommandPacket(const command_packet_t& packet) {
    mc_packet_t packet_to_send;
    packet_to_send.sync1 = 'M';
    packet_to_send.sync2 = 'C';
    packet_to_send.type = PACKET_CMD;
    packet_to_send.length = CMD_PACKET_SIZE;
    packet_to_send.command_packet = swapCommandPacket(packet);

    //Add the checksum
    mc_crc_t crc_calc;
    crc_calc.process_bytes(&packet_to_send, CMD_PACKET_SIZE - 2);
    uint16_t checksum = crc_calc.checksum();
    packet_to_send.payload[CMD_PACKET_SIZE-2] = (checksum & 0x00ff);
    packet_to_send.payload[CMD_PACKET_SIZE-1] = ((checksum >> 8) & 0x00ff);

    std::vector<boost::asio::const_buffer> buffers_to_send;
    buffers_to_send.push_back(boost::asio::const_buffer(&packet_to_send.sync1, 1));
    buffers_to_send.push_back(boost::asio::const_buffer(&packet_to_send.sync2, 1));
    buffers_to_send.push_back(boost::asio::const_buffer(&packet_to_send.type, 1));
    buffers_to_send.push_back(boost::asio::const_buffer(&packet_to_send.length, 1));
    buffers_to_send.push_back(boost::asio::const_buffer(&packet_to_send.payload, packet_to_send.length - 4));
    size_t bytes_sent = boost::asio::write(serial_, buffers_to_send);
    return (bytes_sent == CMD_PACKET_SIZE);
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
          //TODO: check the checksum
          mc_crc_t crc_calc;
          crc_calc.process_bytes(&temp_packet, temp_packet.length);
          if(crc_calc.checksum() == good_checksum) {
            switch(temp_packet.type) {
              case PACKET_STATUS:
                swapStatusPacket(temp_packet.status_packet);
                got_valid_packet = true;
                break;
              case PACKET_ENCODER:
                swapEncoderPacket(temp_packet.encoder_packet);
                got_valid_packet = true;
                break;
              default:
                got_valid_packet = false;
                break;
            }
          }
          else {
            std::cout << "Bad checksum received!" << std::endl;
            got_valid_packet = false;
          }
        }
      }
    }
  }

  //Support swapping command packets from MC to host
  command_packet_t MCBase::swapCommandPacket(const command_packet_t& packet) {
    command_packet_t temp = packet;
    for(size_t i = 0; i < 6; i++) {
      temp.wheel_vels_mmps[i] = htobe16(temp.wheel_vels_mmps[i]);
    }
    return packet;
  }

  void MCBase::swapStatusPacket(status_packet_t& packet) {
    for(size_t i = 0; i < 6; i++) {
      packet.current_mA[i] = be16toh(packet.current_mA[i]);
    }
    packet.battery_current_mA = be16toh(packet.battery_current_mA);
    packet.battery_voltage_cV = be16toh(packet.battery_voltage_cV);
    packet.time = be16toh(packet.time);
  }

  void MCBase::swapEncoderPacket(encoder_packet_t& packet) {
    for(size_t i = 0; i < 6; i++) {
      packet.positions_ticks[i] = be32toh(packet.positions_ticks[i]);
    }
  }

}; //namespace minicrusher_base
