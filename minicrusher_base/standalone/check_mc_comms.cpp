#include <minicrusher_base/mc_base.h>
#include <iostream>

int main(int argc, char* argv[])
{
  std::string port("/dev/ttyUSB0");
  uint32_t baud_rate(115200);
  boost::asio::io_service io;
  minicrusher_base::MCBase mc(port, baud_rate, io); 

  minicrusher_base::mc_packet_t packet;
  for(int i = 0; i < 100; i++)
  {
    std::cout << "Trying to receiving a packet..." << std::endl;
    mc.receivePacket(packet);
    if(packet.type == minicrusher_base::PACKET_STATUS)
    {
        std::cout << "Got status packet" << std::endl;
        std::cout << "Battery voltage (cV), (v): " << packet.status_packet.battery_voltage_cV << ", " << (double) packet.status_packet.battery_voltage_cV / 100. << std::endl;
    }
  }

  std::cout << "Got 100 packets..." << std::endl;
  std::cout << "Trying to send some velocity commands..." << std::endl;

  minicrusher_base::command_packet_t cmd_packet;
  cmd_packet.wheel_vels_mmps[0] = 500;
  cmd_packet.wheel_vels_mmps[1] = -500;
  cmd_packet.wheel_vels_mmps[2] = 500;
  cmd_packet.wheel_vels_mmps[3] = -500;
  cmd_packet.wheel_vels_mmps[4] = 500;
  cmd_packet.wheel_vels_mmps[5] = -500;
  cmd_packet.enable = 1;


  for(int i = 0; i < 100; i++)
  {
    mc.sendCommandPacket(cmd_packet);
    usleep(50 * 1000 );
    std::cout << "Sent a command packet..." << std::endl;
  }

  return 0;
}
