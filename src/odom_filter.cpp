#include <ssugv_bringup/odom_filter.hpp>

using namespace std::chrono_literals;

OdomFilter::OdomFilter() : 
  Node("odom_filter"),
  timeout_value_s_(1.0),
  imu_tf_initialized_(false),
  left_wheel_joint_position_(0.0),
  right_wheel_joint_position_(0.0),
  prev_left_wheel_joint_position_(0.0),
  prev_right_wheel_joint_position_(0.0)
{
    // Declare parameters
    this->declare_parameter("publish_rate", 1.0);
    this->declare_parameter("imu_topic", "imu/data");
    this->declare_parameter("joint_state_topic", "dynamic_joint_states");
    this->declare_parameter("odom_topic", "odometry");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_link_frame", "base_link");

    this->declare_parameter("left_wheel_joint_name", "front_left_wheel_joint");
    this->declare_parameter("right_wheel_joint_name", "front_right_wheel_joint");
    this->declare_parameter("wheel_radius", 0.03);

    // Get Parameters
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string joint_state_topic = this->get_parameter("joint_state_topic").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_link_frame_ = this->get_parameter("base_link_frame").as_string();

    double publish_rate = this->get_parameter("publish_rate").as_double();
    left_wheel_joint_name_ = this->get_parameter("left_wheel_joint_name").as_string();
    right_wheel_joint_name_ = this->get_parameter("right_wheel_joint_name").as_string();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();

    // Initialize tf
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    fused_tf_.setIdentity();
    fused_tf_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    base_to_imu_tf_.setIdentity();
    base_to_imu_tf_.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));

    // Initialize messages
    tf_msg_.header.frame_id = odom_frame_;
    tf_msg_.child_frame_id = base_link_frame_;
    odom_msg_.header.frame_id = odom_frame_;
    odom_msg_.child_frame_id = base_link_frame_;

    // Create publishers and subscribers
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 100);
    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 100,
                                        std::bind(&OdomFilter::imu_callback, this, std::placeholders::_1));
    joint_state_subscriber_ = this->create_subscription<control_msgs::msg::DynamicJointState>(joint_state_topic, 100,
                                        std::bind(&OdomFilter::joint_state_callback, this, std::placeholders::_1));

    // Create timers
    // filter_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0/publish_rate),
    //                             std::bind(&OdomFilter::publish_odom, this));
}

void OdomFilter::imu_callback(sensor_msgs::msg::Imu msg)
{
  std::unique_lock lock(mutex_);

  if (prev_imu_.header.stamp.sec == 0)
  {
    // Initialize IMU
    prev_imu_ = msg;
  }
  else
  {
    imu_ = msg;
  }

  /* Get IMU to base link transform */
  if (!imu_tf_initialized_)
  {
    ///////////////////////////
    base_to_imu_tf_.setIdentity();
    imu_tf_initialized_ = true;
    return;
    ///////////////////////////

    // Release mutex for tflookup time
    lock.unlock();
    try 
    {
      geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(base_link_frame_, imu_.header.frame_id, tf2::TimePointZero);
      
      lock.lock();
      base_to_imu_tf_.setOrigin(tf2::Vector3( tf.transform.translation.x, 
                                              tf.transform.translation.y, 
                                              tf.transform.translation.z));
      base_to_imu_tf_.setRotation(tf2::Quaternion(tf.transform.rotation.x, 
                                                  tf.transform.rotation.y,
                                                  tf.transform.rotation.z,
                                                  tf.transform.rotation.w));
      imu_tf_initialized_ = true;
    } 
    catch (const tf2::TransformException & ex) 
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_PERIOD,
                 "Could not transform %s to %s: %s", imu_.header.frame_id.c_str(), base_link_frame_.c_str(), ex.what());
    }
  }
}

void OdomFilter::joint_state_callback(control_msgs::msg::DynamicJointState msg)
{
  std::unique_lock lock(mutex_);
  
  if (get_position(msg, left_wheel_joint_name_, left_wheel_joint_position_) &&
      get_position(msg, right_wheel_joint_name_, right_wheel_joint_position_))
  {
    if (prev_joint_state_stamp_.seconds() == 0)
    {
      // Initializing previous state
      prev_joint_state_stamp_ = msg.header.stamp;
      return;
    }
    else
    {
      // Updating current state
      joint_state_stamp_ = msg.header.stamp;
      publish_odom();
    }
  }
}

void OdomFilter::publish_odom()
{
  /* Check if IMU transform initialized */
  if (!imu_tf_initialized_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_PERIOD, "IMU to base link transform lookup failure!");
    return;
  }

  /* Check if IMU messages are updated */
  if ( (this->get_clock()->now() - rclcpp::Duration::from_seconds(timeout_value_s_)) > imu_.header.stamp )
  {
    if (imu_.header.stamp.sec != 0)
    {
      // IMU message is initialized, but delays encountered
      RCLCPP_WARN(this->get_logger(), "IMU messages not received for longer than %.3f seconds", timeout_value_s_ );
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), THROTTLE_PERIOD, "No IMU message received");
    }
    return;
  }
  
  // Calculate translation
  double dx = (left_wheel_joint_position_ + right_wheel_joint_position_ - 
                  prev_left_wheel_joint_position_ - prev_right_wheel_joint_position_) * 0.5 * wheel_radius_;
  double dt = (joint_state_stamp_ - prev_joint_state_stamp_).seconds();

  // Calculate translation in base frame
  tf2::Transform heading_tf = fused_tf_;
  heading_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Vector3 base_translation = heading_tf * tf2::Vector3(dx, 0.0, 0.0);

  tf2::Transform dtf;
  dtf.setIdentity();
  dtf.setOrigin(tf2::Vector3(dx, 0.0, 0.0));

  // update translation
  fused_tf_ = fused_tf_ * dtf;
  // update translation variable history
  prev_left_wheel_joint_position_ = left_wheel_joint_position_;
  prev_right_wheel_joint_position_ = right_wheel_joint_position_;
  prev_joint_state_stamp_ = joint_state_stamp_;
  // update messages
  tf_msg_.header.stamp = imu_.header.stamp; // this->get_clock()->now();
  tf_msg_.header.stamp.sec -= 10;
  tf2::Vector3 origin = fused_tf_.getOrigin();
  tf_msg_.transform.translation.x = origin[0];
  tf_msg_.transform.translation.y = origin[1];
  tf_msg_.transform.translation.z = origin[2];

  odom_msg_.header.stamp = imu_.header.stamp; // this->get_clock()->now();
  odom_msg_.header.stamp.sec -= 10;
  odom_msg_.pose.pose.position.x = origin[0];
  odom_msg_.pose.pose.position.y = origin[1];
  odom_msg_.pose.pose.position.z = origin[2];
  odom_msg_.twist.twist.linear.x = 0.5 * base_translation[0] / dt + 0.5 * odom_msg_.twist.twist.linear.x;
  odom_msg_.twist.twist.linear.y = 0.5 * base_translation[1] / dt + 0.5 * odom_msg_.twist.twist.linear.y;
  odom_msg_.twist.twist.linear.z = 0.5 * base_translation[2] / dt + 0.5 * odom_msg_.twist.twist.linear.z;

  std::cout << base_translation[0] << "  " << base_translation[1] << "  " << dt << std::endl;

  // Calculate IMU message interval
  double dt_imu = (rclcpp::Time(imu_.header.stamp) - rclcpp::Time(prev_imu_.header.stamp)).seconds();
  if (dt_imu > 0)
  {
    // Transform IMU values to base frame
    tf2::Transform imu_tf;
    imu_tf.setRotation(tf2::Quaternion(imu_.orientation.x, imu_.orientation.y, imu_.orientation.z, imu_.orientation.w));
    imu_tf = base_to_imu_tf_ * imu_tf;

    // Extract yaw rotation
    tf2::Quaternion orientation, q;
    q = imu_tf.getRotation();
    double q_w_sign = (q.getW() >= 0)? 1.0 : -1.0;
    double q_z = q.getZ();
    orientation.setValue(0.0, 0.0, q_z, sqrt(1.0 - pow(q_z, 2)) * q_w_sign);

    // Calculate yaw change
    imu_tf.setRotation(orientation);
    double drot = (fused_tf_.inverse() * imu_tf).getRotation().getAngle();

    // update rotation
    fused_tf_.setRotation(orientation);
    // update rotation variable history
    prev_imu_ = imu_;
    // update messages
    tf_msg_.transform.rotation.w = orientation.getW();
    tf_msg_.transform.rotation.x = orientation.getX();
    tf_msg_.transform.rotation.y = orientation.getY();
    tf_msg_.transform.rotation.z = orientation.getZ();

    odom_msg_.pose.pose.orientation = tf_msg_.transform.rotation;
    odom_msg_.twist.twist.angular.z = drot / dt_imu;
  }

  // Publish transform
  tf_broadcaster_->sendTransform(tf_msg_);

  // Publish odometry
  odom_publisher_->publish(odom_msg_);
}

bool OdomFilter::get_position(control_msgs::msg::DynamicJointState msg, std::string joint_name, double &position)
{
  bool success = false;
  auto index = std::find(msg.joint_names.begin(), msg.joint_names.end(), joint_name);
  if (index != msg.joint_names.end())
  {
    control_msgs::msg::InterfaceValue values= msg.interface_values.at(index - msg.joint_names.begin());
    auto interface_index = std::find(values.interface_names.begin(), values.interface_names.end(), "position");
    if (interface_index == values.interface_names.end())
    {
      RCLCPP_ERROR(this->get_logger(), "%s does not contain a 'position' field! Check joint_state_topic.", joint_name.c_str());
    }
    else
    {
      position = values.values.at(interface_index - values.interface_names.begin());
      success = true;
    }
  }
  return success;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFilter>());
  rclcpp::shutdown();
  return 0;
}