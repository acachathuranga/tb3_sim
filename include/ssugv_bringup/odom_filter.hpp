#ifndef ODOM_FILTER_H
#define ODOM_FILTER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <mutex>
#include <algorithm>
#include <math.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#define THROTTLE_PERIOD 10000

class OdomFilter : public rclcpp::Node
{
  public:
    OdomFilter();

  private:
    rclcpp::TimerBase::SharedPtr filter_timer_;
    const double timeout_value_s_;

    std::string left_wheel_joint_name_;
    std::string right_wheel_joint_name_;
    double wheel_radius_;

    std::string odom_frame_;
    std::string base_link_frame_;
    tf2::Transform base_to_imu_tf_;
    bool imu_tf_initialized_;


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::mutex mutex_;
    rclcpp::Time joint_state_stamp_;
    rclcpp::Time prev_joint_state_stamp_;
    double left_wheel_joint_position_;
    double right_wheel_joint_position_;
    double prev_left_wheel_joint_position_;
    double prev_right_wheel_joint_position_;
    sensor_msgs::msg::Imu imu_;
    sensor_msgs::msg::Imu prev_imu_;

    tf2::Transform fused_tf_;

    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::TransformStamped tf_msg_;

    void imu_callback(sensor_msgs::msg::Imu msg);
    void joint_state_callback(control_msgs::msg::DynamicJointState msg);

    bool get_position(control_msgs::msg::DynamicJointState msg, std::string joint_name, double &position);

    void publish_odom();
};

#endif // ODOM_FILTER_H