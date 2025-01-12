#include "mowbot/gnss_fuser/gnss_fuser_node.hpp"

#include <mowbot_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include <GeographicLib/LocalCartesian.hpp>

namespace mowbot::gnss_fuser
{
GNSSFuser::GNSSFuser(const rclcpp::NodeOptions & node_options)
: Node("gnss_fuser", node_options),
  tf2_listener_(tf2_buffer_),
  gnss_base_frame_(declare_parameter<std::string>("gnss_base_frame")),
  gnss_left_frame_(declare_parameter<std::string>("gnss_left_frame")),
  gnss_right_frame_(declare_parameter<std::string>("gnss_right_frame")),
  publish_freq_(declare_parameter<float>("publish_freq")),
  use_gnss_ins_orientation_msg_(declare_parameter<bool>("use_gnss_ins_orientation_msg")),
  use_imu_orientation_msg_(declare_parameter<bool>("use_imu_orientation_msg"))
{
  RCLCPP_INFO(this->get_logger(), "GNSS Fuser node started");
  RCLCPP_INFO(this->get_logger(), "GNSS base frame: %s", gnss_base_frame_.c_str());

  gnss_left_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "left/fix", rclcpp::QoS(1),
    std::bind(&GNSSFuser::callback_gnss_left, this, std::placeholders::_1));
  gnss_right_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "right/fix", rclcpp::QoS(1),
    std::bind(&GNSSFuser::callback_gnss_right, this, std::placeholders::_1));
  
  if (publish_freq_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Publish frequency must be greater than zero.");
    throw std::runtime_error("Publish frequency must be greater than zero.");
  }
  gnss_fuse_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000 / publish_freq_)), 
    std::bind(&GNSSFuser::callback_gnss_fuse_timer, this));

  fused_gnss_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("gnss_fused/fix", rclcpp::QoS(1));

  if (use_gnss_ins_orientation_msg_) {
    gnss_ins_orientation_pub_ = create_publisher<mowbot_sensing_msgs::msg::GnssInsOrientationStamped>(
      "gnss_fused/gnss_ins_orientation", rclcpp::QoS(1));
  }
  if (use_imu_orientation_msg_) {
    imu_orientation_pub_ = create_publisher<sensor_msgs::msg::Imu>("gnss_fused/imu_orientation", rclcpp::QoS(1));
  }
}

void GNSSFuser::callback_gnss_left(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  //check fixed topic
  const bool is_status_fixed = is_fixed(msg->status);
  if (!is_status_fixed) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Left GNSS Is Not Fixed Topic. Skipping Process.");
    return;
  }
  // check msg frame
  if (msg->header.frame_id != gnss_left_frame_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Left GNSS Frame ID is not " << gnss_left_frame_.c_str() << ". Skipping Process.");
    return;
  }
  gnss_left_msg_.reset(new sensor_msgs::msg::NavSatFix);
  *gnss_left_msg_ = *msg;
}

void GNSSFuser::callback_gnss_right(
    const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  //check fixed topic
  const bool is_status_fixed = is_fixed(msg->status);
  if (!is_status_fixed) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Right GNSS Is Not Fixed Topic. Skipping Process.");
    return;
  }
  // check msg frame
  if (msg->header.frame_id != gnss_right_frame_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Right GNSS Frame ID is not " << gnss_right_frame_.c_str() << ". Skipping Process.");
    return;
  }
  gnss_right_msg_.reset(new sensor_msgs::msg::NavSatFix);
  *gnss_right_msg_ = *msg;
}

void GNSSFuser::callback_gnss_fuse_timer()
{
  // Check if GNSS messages are available
  if (!gnss_left_msg_ || !gnss_right_msg_) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Left or Right GNSS data is not available. Skipping Process.");
    return;
  }

  // Use the left GNSS as the reference point (WGS84 origin)
  double origin_lat = gnss_left_msg_->latitude;
  double origin_lon = gnss_left_msg_->longitude;
  double origin_alt = gnss_left_msg_->altitude;

  // Initialize LocalCartesian (ENU) converter with the origin
  GeographicLib::LocalCartesian enu_converter(origin_lat, origin_lon, origin_alt);

  // Convert left GNSS to ENU coordinates
  double x_left, y_left, z_left;
  enu_converter.Forward(
    gnss_left_msg_->latitude, gnss_left_msg_->longitude, gnss_left_msg_->altitude,
    x_left, y_left, z_left);

  // Convert right GNSS to ENU coordinates
  double x_right, y_right, z_right;
  enu_converter.Forward(
    gnss_right_msg_->latitude, gnss_right_msg_->longitude, gnss_right_msg_->altitude,
    x_right, y_right, z_right);

  // calculate enu heading angle based on left and right gnss data
  double heading = atan2(y_right - y_left, x_right - x_left);
  RCLCPP_INFO(this->get_logger(), "Heading: %.2f", heading);

  // convert heading angle to quaternion
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, heading);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(orientation);

  // publish gnss ins orientation
  if (use_gnss_ins_orientation_msg_) {
    mowbot_sensing_msgs::msg::GnssInsOrientationStamped gnss_ins_orientation_msg;
    gnss_ins_orientation_msg.header.stamp = this->now();
    gnss_ins_orientation_msg.header.frame_id = gnss_base_frame_;
    gnss_ins_orientation_msg.orientation.orientation.x = quat_msg.x;
    gnss_ins_orientation_msg.orientation.orientation.y = quat_msg.y;
    gnss_ins_orientation_msg.orientation.orientation.z = quat_msg.z;
    gnss_ins_orientation_msg.orientation.orientation.w = quat_msg.w;
    // Set msg_gnss_ins_orientation_stamped_ with temporary values (not to publish zero value
    // covariances)
    gnss_ins_orientation_msg.orientation.rmse_rotation_x = 1.0;
    gnss_ins_orientation_msg.orientation.rmse_rotation_y = 1.0;
    gnss_ins_orientation_msg.orientation.rmse_rotation_z = 1.0;
    gnss_ins_orientation_pub_->publish(gnss_ins_orientation_msg);
  }

  // publish imu orientation
  if (use_imu_orientation_msg_) {
    sensor_msgs::msg::Imu imu_orientation_msg{};
    imu_orientation_msg.header.stamp = this->now();
    imu_orientation_msg.header.frame_id = gnss_base_frame_;
    imu_orientation_msg.orientation.x = quat_msg.x;
    imu_orientation_msg.orientation.y = quat_msg.y;
    imu_orientation_msg.orientation.z = quat_msg.z;
    imu_orientation_msg.orientation.w = quat_msg.w;
    imu_orientation_msg.orientation_covariance = {
      0.0479, 0.0, 0.0,
      0.0, 0.020, 0.0,
      0.0, 0.0, 0.0041
    };
    imu_orientation_pub_->publish(imu_orientation_msg);
  }

  // get TF from left gnss frame to gnss_base_frame
  auto tf_gnss_left2base_msg_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  get_static_transform(
    gnss_left_msg_->header.frame_id, gnss_base_frame_, tf_gnss_left2base_msg_ptr, gnss_left_msg_->header.stamp);
  tf2::Transform tf_gnss_left2base{};
  tf2::fromMsg(tf_gnss_left2base_msg_ptr->transform, tf_gnss_left2base);

  // calculate gnss data at gnss_base_frame based on left gnss data
  tf2::Vector3 gnss_left_enu(x_left, y_left, z_left);
  tf2::Vector3 gnss_left_at_base_frame = tf_gnss_left2base * gnss_left_enu;

  // get TF from right gnss frame to gnss_base_frame
  auto tf_gnss_right2base_msg_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  get_static_transform(
    gnss_right_msg_->header.frame_id, gnss_base_frame_, tf_gnss_right2base_msg_ptr, gnss_right_msg_->header.stamp);
  tf2::Transform tf_gnss_right2base{};
  tf2::fromMsg(tf_gnss_right2base_msg_ptr->transform, tf_gnss_right2base);

  // calculate gnss data at gnss_base_frame based on right gnss data
  tf2::Vector3 gnss_right_enu(x_right, y_right, z_right);
  tf2::Vector3 gnss_right_at_base_frame = tf_gnss_right2base * gnss_right_enu;

  // calculate fused gnss data at gnss_base_frame
  tf2::Vector3 gnss_fused_at_base_frame = (gnss_left_at_base_frame + gnss_right_at_base_frame) / 2.0;

  double fused_lat, fused_lon, fused_alt;
  enu_converter.Reverse(
    gnss_fused_at_base_frame.x(), gnss_fused_at_base_frame.y(), gnss_fused_at_base_frame.z(),
    fused_lat, fused_lon, fused_alt);

  // publish fused gnss data
  sensor_msgs::msg::NavSatFix fused_gnss_msg;
  fused_gnss_msg.header.stamp = this->now();
  fused_gnss_msg.header.frame_id = gnss_base_frame_;
  fused_gnss_msg.status = gnss_left_msg_->status;
  fused_gnss_msg.latitude = fused_lat;
  fused_gnss_msg.longitude = fused_lon;
  fused_gnss_msg.altitude = fused_alt;
  for (int i = 0; i < 9; i++) {
      fused_gnss_msg.position_covariance[i] =
        (gnss_left_msg_->position_covariance[i] + gnss_right_msg_->position_covariance[i]) / 2.0;
  }
  fused_gnss_msg.position_covariance_type = gnss_left_msg_->position_covariance_type;
  fused_gnss_pub_->publish(fused_gnss_msg);

  // Print left GNSS data (raw WGS84 coordinates)
  RCLCPP_INFO(this->get_logger(), "GNSS left (WGS84): Lat: %.8f, Lon: %.8f, Alt: %.2f",
                gnss_left_msg_->latitude, gnss_left_msg_->longitude, gnss_left_msg_->altitude);
  
  // Print right GNSS data (raw WGS84 coordinates)
  RCLCPP_INFO(this->get_logger(), "GNSS right (WGS84): Lat: %.8f, Lon: %.8f, Alt: %.2f",
              gnss_right_msg_->latitude, gnss_right_msg_->longitude, gnss_right_msg_->altitude);
              
  // Print fused GNSS data (WGS84 coordinates)
  RCLCPP_INFO(this->get_logger(), "GNSS fused (WGS84): Lat: %.8f, Lon: %.8f, Alt: %.2f",
              fused_lat, fused_lon, fused_alt);
}

bool GNSSFuser::is_fixed(
    const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg)
{
  return nav_sat_status_msg.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

bool GNSSFuser::get_static_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr,
  const builtin_interfaces::msg::Time & stamp)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr = tf2_buffer_.lookupTransform(
      target_frame, source_frame,
      tf2::TimePoint(std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(), ex.what());
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str());

    transform_stamped_ptr->header.stamp = stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

}  // namespace mowbot::gnss_fuser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mowbot::gnss_fuser::GNSSFuser)