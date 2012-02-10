#include <ros/ros.h>
#include <minicrusher_base/mc_base.h>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Twist.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

namespace minicrusher_base
{
  class MCDriverNode
  {
    public:
      MCDriverNode();
      ~MCDriverNode();
      void run();
    private:
      ros::NodeHandle nh_, priv_nh_;
      std::string port_;
      int baud_rate_;
      double track_width_m_;
      boost::asio::io_service io_;
      boost::shared_ptr<MCBase> mc_ptr_;

      status_packet_t last_status_;

      diagnostic_updater::Updater updater_;

      ros::Subscriber twist_sub_;
      ros::Publisher encoder_pub_;

      void twistCallback(const geometry_msgs::Twist::ConstPtr& twist);
      void setupDiagnostics();

      void checkStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);
  };

  MCDriverNode::MCDriverNode() :
    priv_nh_("~"),
    io_()
  {

    priv_nh_.param("port", port_, std::string("/dev/ttyUSB0"));
    priv_nh_.param("baud_rate", baud_rate_, 115200);
    priv_nh_.param("track_width_m", track_width_m_, 0.508); // ~20inches

    mc_ptr_ = boost::make_shared<MCBase>(port_, baud_rate_, boost::ref(io_));

    twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&MCDriverNode::twistCallback, this, _1));
    
    setupDiagnostics();
  }

  void MCDriverNode::setupDiagnostics()
  {
    updater_.setHardwareID("Minicrusher 01");

    updater_.add("Status", this, &MCDriverNode::checkStatus);
  }

  void MCDriverNode::checkStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
  }

  void MCDriverNode::twistCallback(const geometry_msgs::Twist::ConstPtr& twist)
  {
    command_packet_t cmd_packet;
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

    if(!mc_ptr_->sendCommandPacket(cmd_packet))
    {
      ROS_WARN("Failed to send command packet to MC");
    }
  }

}; //namespace minicrusher_base

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "minicrusher_driver_node");

  try{
    ros::spin();
  }
  catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating MCBase. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}
