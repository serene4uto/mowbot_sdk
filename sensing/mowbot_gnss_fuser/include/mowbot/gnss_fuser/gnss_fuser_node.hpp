// File: gnss_fuser_node.hpp

#ifndef MOWBOT__GNSS_FUSER__GNSS_FUSER_NODE_HPP_
#define MOWBOT__GNSS_FUSER__GNSS_FUSER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <mowbot_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>


#include <vector>
#include <string>

namespace mowbot::gnss_fuser
{
class GNSSFuser : public rclcpp::Node
{
public:
  explicit GNSSFuser(const rclcpp::NodeOptions & node_options);

private:

  void callback_gnss_left(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void callback_gnss_right(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void callback_gnss_fuse_timer();

  static bool is_fixed(
    const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg);
  bool get_static_transform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr,
    const builtin_interfaces::msg::Time & stamp);
    
  tf2::BufferCore tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_right_sub_;
  rclcpp::TimerBase::SharedPtr gnss_fuse_timer_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fused_gnss_pub_;
  rclcpp::Publisher<mowbot_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr gnss_ins_orientation_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_orientation_pub_;

  const std::string gnss_base_frame_;
  const std::string gnss_left_frame_;
  const std::string gnss_right_frame_;
  const float publish_freq_;
  const bool use_gnss_ins_orientation_msg_;
  const bool use_imu_orientation_msg_;


  sensor_msgs::msg::NavSatFix::SharedPtr gnss_left_msg_;
  sensor_msgs::msg::NavSatFix::SharedPtr gnss_right_msg_;
};
} // namespace mowbot

#endif  // MOWBOT__GNSS_FUSER__GNSS_FUSER_NODE_HPP_