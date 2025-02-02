#ifndef WHEEL_STEERABLE_PLUGIN_HH
#define WHEEL_STEERABLE_PLUGIN_HH

#include <map>
#include <memory>
#include <string>
#include <vector>

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <control_msgs/msg/joint_controller_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Standard
#include <mutex>

namespace gazebo
{
  class GazeboRosWheelsSteerable : public ModelPlugin
  {
    enum Wheel
    {
      FRONT_LEFT = 0,
      FRONT_RIGHT = 1,
      REAR_LEFT = 2,
      REAR_RIGHT = 3,
    };

  public:
    GazeboRosWheelsSteerable();
    ~GazeboRosWheelsSteerable();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
    void Reset() override;

  protected:
    void UpdateChild();
    void FiniChild();
    void UpdateOdometryEncoder();
    void PublishOdometry();
    void GetGroundTruth();
    void SetCovariance();

  private:
    // Gazebo
    physics::ModelPtr parent_;
    event::ConnectionPtr update_connection_;
    std::vector<physics::JointPtr> joints_rotation_;

    // ROS2 Node
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_odometry_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

    // Publishers for debugging
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_angle_setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_angle_setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_left_angle_current_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr front_right_angle_current_pub_;

    geometry_msgs::msg::Twist::SharedPtr cmd_twist_;

    std::mutex lock_;
    bool alive_;

    // Parameters
    std::string topic_cmd_twist_;
    std::string topic_odom_;
    std::string frame_odom_;
    std::string topic_ground_truth_;
    std::string frame_ground_truth_;
    std::string namespace_;
    std::string frame_base_;
    std::string joint_rear_left_;
    std::string joint_rear_right_;
    std::string joint_steering_left_;
    std::string joint_steering_right_;
    bool publishOdomTF_;
    double wheelbase_distance_;
    double kingpin_distance_;
    double max_steering_angle_;
    double torque_max_wheel_;
    double wheel_radius_;
    double update_rate_controller_;
    double update_period_controller_;

    // PID Controller Parameters
    double max_effort_pid_;
    double pid_p_;
    double pid_i_;
    double pid_d_;

    struct PID_Controller_State
    {
      double max_effort_pid_;
      double pid_p_;
      double pid_i_;
      double pid_d_;

      double integral_;
      double pre_error_;
    };

    PID_Controller_State pid_controller_front_left_;
    PID_Controller_State pid_controller_front_right_;

    // Odometry
    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::Pose2D pose_encoder_;
    common::Time last_odom_update_;
    common::Time next_update_time_;
    common::Time last_pid_update_time_;

    // Helper Functions
    void OnUpdate();
    void callbackTopicCMD(const geometry_msgs::msg::Twist::SharedPtr cmd_msg);
    void setPIDParameters(PID_Controller_State &state, double p, double i, double d, double maxEffort);
    double calculatePID(PID_Controller_State &state, double setValue, double currentValue, double dt);
  };
}

#endif
