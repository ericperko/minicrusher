#include <ros/ros.h>
#include <minicrusher_base/mc_base.h>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Twist.h>
#include <minicrusher_msgs/Encoder.h>
#include <minicrusher_msgs/EncoderArray.h>
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
      ros::NodeHandle nh_, priv_nh_, encoder_nh_;

      // Parameters
      std::string port_;
      int baud_rate_;
      double track_width_m_;
      double lf_enc_m_per_tick_, lm_enc_m_per_tick_, lb_enc_m_per_tick_, rf_enc_m_per_tick_, rm_enc_m_per_tick_, rb_enc_m_per_tick_;

      boost::asio::io_service io_;
      boost::shared_ptr<MCBase> mc_ptr_;

      status_packet_t last_status_;

      double expected_encoder_pub_rate_;

      diagnostic_updater::Updater updater_;
      diagnostic_updater::DiagnosedPublisher<minicrusher_msgs::Encoder> lf_encoder_pub_;
      diagnostic_updater::DiagnosedPublisher<minicrusher_msgs::Encoder> lm_encoder_pub_;
      diagnostic_updater::DiagnosedPublisher<minicrusher_msgs::Encoder> lb_encoder_pub_;
      diagnostic_updater::DiagnosedPublisher<minicrusher_msgs::Encoder> rf_encoder_pub_;
      diagnostic_updater::DiagnosedPublisher<minicrusher_msgs::Encoder> rm_encoder_pub_;
      diagnostic_updater::DiagnosedPublisher<minicrusher_msgs::Encoder> rb_encoder_pub_;

      ros::Publisher encoder_array_pub_;
      ros::Subscriber twist_sub_;

      void twistCallback(const geometry_msgs::Twist::ConstPtr& twist);
      void setupDiagnostics();
      void updateDiagnostics();

      void checkStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);
      void publishEncoderPacket(const encoder_packet_t& encoder_packet);
  };

  MCDriverNode::MCDriverNode() :
    priv_nh_("~"),
    encoder_nh_("encoders"),
    io_(),
    expected_encoder_pub_rate_(100.0),
    lf_encoder_pub_(encoder_nh_.advertise<minicrusher_msgs::Encoder>("left_front", 1),
        updater_, diagnostic_updater::FrequencyStatusParam(&expected_encoder_pub_rate_, &expected_encoder_pub_rate_, 3.0, 5),
        diagnostic_updater::TimeStampStatusParam()),
    lm_encoder_pub_(encoder_nh_.advertise<minicrusher_msgs::Encoder>("left_middle", 1),
        updater_, diagnostic_updater::FrequencyStatusParam(&expected_encoder_pub_rate_, &expected_encoder_pub_rate_, 3.0, 5),
        diagnostic_updater::TimeStampStatusParam()),
    lb_encoder_pub_(encoder_nh_.advertise<minicrusher_msgs::Encoder>("left_back", 1),
        updater_, diagnostic_updater::FrequencyStatusParam(&expected_encoder_pub_rate_, &expected_encoder_pub_rate_, 3.0, 5),
        diagnostic_updater::TimeStampStatusParam()),
    rf_encoder_pub_(encoder_nh_.advertise<minicrusher_msgs::Encoder>("right_front", 1),
        updater_, diagnostic_updater::FrequencyStatusParam(&expected_encoder_pub_rate_, &expected_encoder_pub_rate_, 3.0, 5),
        diagnostic_updater::TimeStampStatusParam()),
    rm_encoder_pub_(encoder_nh_.advertise<minicrusher_msgs::Encoder>("right_middle", 1),
        updater_, diagnostic_updater::FrequencyStatusParam(&expected_encoder_pub_rate_, &expected_encoder_pub_rate_, 3.0, 5),
        diagnostic_updater::TimeStampStatusParam()),
    rb_encoder_pub_(encoder_nh_.advertise<minicrusher_msgs::Encoder>("right_back", 1),
        updater_, diagnostic_updater::FrequencyStatusParam(&expected_encoder_pub_rate_, &expected_encoder_pub_rate_, 3.0, 5),
        diagnostic_updater::TimeStampStatusParam())
  {

    priv_nh_.param("port", port_, std::string("/dev/ttyUSB0"));
    priv_nh_.param("baud_rate", baud_rate_, 115200);
    priv_nh_.param("track_width_m", track_width_m_, 0.508); // ~20inches

    ros::NodeHandle priv_enc_nh(priv_nh_, "encoders");
    priv_enc_nh.param("left_front_m_per_tick", lf_enc_m_per_tick_, 1.0);
    priv_enc_nh.param("left_middle_m_per_tick", lm_enc_m_per_tick_, 1.0);
    priv_enc_nh.param("left_back_m_per_tick", lb_enc_m_per_tick_, 1.0);
    priv_enc_nh.param("right_front_m_per_tick", rf_enc_m_per_tick_, 1.0);
    priv_enc_nh.param("right_middle_m_per_tick", rm_enc_m_per_tick_, 1.0);
    priv_enc_nh.param("right_back_m_per_tick", rb_enc_m_per_tick_, 1.0);

    mc_ptr_ = boost::make_shared<MCBase>(port_, baud_rate_, boost::ref(io_));

    twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&MCDriverNode::twistCallback, this, _1));
    encoder_array_pub_ = encoder_nh_.advertise<minicrusher_msgs::EncoderArray>("array", 1);

    setupDiagnostics();
  }

  MCDriverNode::~MCDriverNode()
  {
  }

  void MCDriverNode::setupDiagnostics()
  {
    updater_.setHardwareID("Minicrusher 01");

    updater_.add("Status", this, &MCDriverNode::checkStatus);
  }

  void MCDriverNode::updateDiagnostics()
  {
    updater_.update();
  }

  void MCDriverNode::checkStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.add("Battery Voltage (V)", last_status_.battery_voltage_cV / 100.0);
  }

  void MCDriverNode::publishEncoderPacket(const encoder_packet_t& packet)
  {
    minicrusher_msgs::EncoderArray encoder_array;
    minicrusher_msgs::Encoder encoder;

    encoder.header.stamp = ros::Time::now();
    encoder.header.frame_id = "left_front_encoder";
    encoder.distance = packet.positions_ticks[0] * lf_enc_m_per_tick_;
    encoder_array.encoders.push_back(encoder);
    lf_encoder_pub_.publish(encoder);

    encoder.header.frame_id = "left_middle_encoder";
    encoder.distance = packet.positions_ticks[2] * lm_enc_m_per_tick_;
    encoder_array.encoders.push_back(encoder);
    lm_encoder_pub_.publish(encoder);

    encoder.header.frame_id = "left_back_encoder";
    encoder.distance = packet.positions_ticks[4] * lb_enc_m_per_tick_;
    encoder_array.encoders.push_back(encoder);
    lb_encoder_pub_.publish(encoder);

    encoder.header.frame_id = "right_front_encoder";
    encoder.distance = packet.positions_ticks[1] * rf_enc_m_per_tick_;
    encoder_array.encoders.push_back(encoder);
    rf_encoder_pub_.publish(encoder);

    encoder.header.frame_id = "right_middle_encoder";
    encoder.distance = packet.positions_ticks[3] * rm_enc_m_per_tick_;
    encoder_array.encoders.push_back(encoder);
    rm_encoder_pub_.publish(encoder);

    encoder.header.frame_id = "right_back_encoder";
    encoder.distance = packet.positions_ticks[5] * rb_enc_m_per_tick_;
    encoder_array.encoders.push_back(encoder);
    rb_encoder_pub_.publish(encoder);

    encoder_array_pub_.publish(encoder_array);
  }

  void MCDriverNode::twistCallback(const geometry_msgs::Twist::ConstPtr& twist)
  {
    command_packet_t cmd_packet;

    double v_change = -1.0 * twist->angular.z * track_width_m_ / 2.0;

    int16_t lefts = ((twist->linear.x + v_change) * 1000);
    int16_t rights = (-(twist->linear.x - v_change) * 1000);

    ROS_DEBUG("Commands in ticks: Left: %d, Right: %d", lefts, rights);

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

  void MCDriverNode::run()
  {
    while(ros::ok())
    {
        mc_packet_t packet;
        mc_ptr_->receivePacket(packet);
        switch(packet.type)
        {
          case(PACKET_ENCODER):
            publishEncoderPacket(packet.encoder_packet);
            break;
          case(PACKET_STATUS):
            last_status_ = packet.status_packet;
            updateDiagnostics();
            break;
        }
        ros::spinOnce();
    }
  }

}; //namespace minicrusher_base

int main(int argc, char* argv[])
{

  ros::init(argc, argv, "minicrusher_driver_node");

  ros::NodeHandle nh;

  try{
    minicrusher_base::MCDriverNode driver;
    driver.run();
  }
  catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating MCBase. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
  return 0;
}
