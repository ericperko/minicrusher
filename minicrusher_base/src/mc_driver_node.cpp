#include <ros/ros.h>
#include <minicrusher_base/mc_base.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist, minicrusher_base::MCBase& mc)
{
  minicrusher_base::command_packet_t cmd_packet;
  int16_t lefts = floor(twist->linear.x * 1000);
  int16_t rights = floor(twist->linear.x * 1000);

  cmd_packet.enable = 1;
  for(uint8_t i = 0; i < 6; i++)
  {
    if((i % 2) == 1)
    {
      cmd_packet.wheel_vels_mmps[i] = rights;
    }
    else
    {
      cmd_packet.wheel_vels_mmps[i] = lefts;
    }
  }

  if(!mc.sendCommandPacket(cmd_packet))
  {
    ROS_WARN("Failed to send command packet to MC");
  }
}

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "minicrusher_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string port;
  int baud_rate;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("baud_rate", baud_rate, 115200);

  boost::asio::io_service io;

  try{
    minicrusher_base::MCBase mc(port, baud_rate, io);
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(cmd_vel_callback, _1, boost::ref(mc)));
    ros::spin();
  }
  catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating MCBase. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}
