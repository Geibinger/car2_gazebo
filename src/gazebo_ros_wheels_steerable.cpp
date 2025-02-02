#include "car2_gazebo/gazebo_ros_wheels_steerable.h"

#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <tf2/LinearMath/Quaternion.h>

namespace gazebo
{

  GazeboRosWheelsSteerable::GazeboRosWheelsSteerable()
  {
  }

  GazeboRosWheelsSteerable::~GazeboRosWheelsSteerable()
  {
    FiniChild();
  }

  void GazeboRosWheelsSteerable::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    this->parent_ = _parent;

    // Initialize ROS node
    ros_node_ = gazebo_ros::Node::Get(_sdf);

    // Read parameters from SDF
    // Get parameters with default values if not specified
    topic_cmd_twist_ = _sdf->Get<std::string>("topicTwist", "cmd_vel").first;
    topic_odom_ = _sdf->Get<std::string>("topicOdom", "odom").first;
    frame_odom_ = _sdf->Get<std::string>("frameOdom", "odom").first;
    publishOdomTF_ = _sdf->Get<bool>("publishOdomTF", true).first;
    topic_ground_truth_ = _sdf->Get<std::string>("topicGroundTruth", "ground_truth").first;
    frame_ground_truth_ = _sdf->Get<std::string>("frameGroundTruth", "ground_truth").first;
    frame_base_ = _sdf->Get<std::string>("frameBase", "base_link").first; // Note: base_link
    joint_rear_left_ = _sdf->Get<std::string>("jointRearLeft", "wheel_axis_rear_left_joint").first;
    joint_rear_right_ = _sdf->Get<std::string>("jointRearRight", "wheel_axis_rear_right_joint").first;
    joint_steering_left_ = _sdf->Get<std::string>("jointSteeringLeft", "wheel_mount_front_left_joint").first;
    joint_steering_right_ = _sdf->Get<std::string>("jointSteeringRight", "wheel_mount_front_right_joint").first;
    torque_max_wheel_ = _sdf->Get<double>("torqueMaxWheel", 5.0).first;
    wheelbase_distance_ = _sdf->Get<double>("wheelbaseDistance", 0.26).first;
    kingpin_distance_ = _sdf->Get<double>("kingpinDistance", 0.15).first;
    max_steering_angle_ = _sdf->Get<double>("maxSteeringAngle", M_PI / 6).first;
    update_rate_controller_ = _sdf->Get<double>("updateRateController", 100.0).first;
    max_effort_pid_ = _sdf->Get<double>("maxEffortSteeringPid", 5.12).first;
    pid_p_ = _sdf->Get<double>("pidP", 5.0).first;
    pid_i_ = _sdf->Get<double>("pidI", 50.0).first;
    pid_d_ = _sdf->Get<double>("pidD", 1.0).first;
    wheel_radius_ = _sdf->Get<double>("wheelRadius", 0.03225).first;
    namespace_ = _sdf->Get<std::string>("namespace", "").first;

    if (!namespace_.empty())
    {
      frame_base_ = namespace_ + "/" + frame_base_;
    }

    RCLCPP_INFO(ros_node_->get_logger(), "Controllable Joints:");
    auto joints = _parent->GetJoints();
    for (const auto &j : joints)
    {
      RCLCPP_INFO(ros_node_->get_logger(), "%s", j->GetName().c_str());
    }
    joints_rotation_.resize(4);

    joints_rotation_[REAR_LEFT] = _parent->GetJoint(joint_rear_left_);
    if (!joints_rotation_[REAR_LEFT])
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Joint not found: %s", joint_rear_left_.c_str());
      return;
    }
    joints_rotation_[REAR_RIGHT] = _parent->GetJoint(joint_rear_right_);
    if (!joints_rotation_[REAR_RIGHT])
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Joint not found: %s", joint_rear_right_.c_str());
      return;
    }

    joints_rotation_[FRONT_LEFT] = _parent->GetJoint(joint_steering_left_);
    if (!joints_rotation_[FRONT_LEFT])
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Joint not found: %s", joint_steering_left_.c_str());
      return;
    }
    joints_rotation_[FRONT_RIGHT] = _parent->GetJoint(joint_steering_right_);
    if (!joints_rotation_[FRONT_RIGHT])
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Joint not found: %s", joint_steering_right_.c_str());
      return;
    }

    joints_rotation_[REAR_LEFT]->SetParam("fmax", 0, torque_max_wheel_);
    joints_rotation_[REAR_RIGHT]->SetParam("fmax", 0, torque_max_wheel_);

    // ROS2 Publishers and Subscribers
    cmd_vel_subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        topic_cmd_twist_, 1,
        std::bind(&GazeboRosWheelsSteerable::callbackTopicCMD, this, std::placeholders::_1));

    odometry_publisher_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(topic_odom_, 1);

    ground_truth_odometry_publisher_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(topic_ground_truth_, 1);

    // Publishers for debugging
    // TODO: They are currently not used and should be removed
    // front_left_angle_setpoint_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>("front_left_angle_setpoint", 1);
    // front_right_angle_setpoint_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>("front_right_angle_setpoint", 1);
    // front_left_angle_current_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>("front_left_angle_current", 1);
    // front_right_angle_current_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64>("front_right_angle_current", 1);

    if (this->update_rate_controller_ > 0.0)
      this->update_period_controller_ = 1.0 / this->update_rate_controller_;
    else
      this->update_period_controller_ = 0.0;
    pose_encoder_.x = 0.0;
    pose_encoder_.y = 0.0;
    pose_encoder_.theta = 0.0;

    // Setup PID controllers for steering
    setPIDParameters(pid_controller_front_left_, pid_p_, pid_i_, pid_d_, max_effort_pid_);
    setPIDParameters(pid_controller_front_right_, pid_p_, pid_i_, pid_d_, max_effort_pid_);

    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    next_update_time_ = parent_->GetWorld()->SimTime();
    last_pid_update_time_ = parent_->GetWorld()->SimTime();

    // Connect to the world update event
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosWheelsSteerable::OnUpdate, this));

    alive_ = true;
  }

  void GazeboRosWheelsSteerable::Reset()
  {
    next_update_time_ = parent_->GetWorld()->SimTime();
    last_pid_update_time_ = parent_->GetWorld()->SimTime();
  }

  void GazeboRosWheelsSteerable::OnUpdate()
  {
    UpdateChild();
  }

  void GazeboRosWheelsSteerable::UpdateChild()
  {
    common::Time current_time = parent_->GetWorld()->SimTime();

    if (next_update_time_ < current_time)
    {
      while (next_update_time_ < current_time)
        next_update_time_ += common::Time(this->update_period_controller_);
      UpdateOdometryEncoder();
      PublishOdometry();

      // Declare variables
      double front_left_angle = 0.0;
      double front_right_angle = 0.0;
      double rear_left_velocity = 0.0;
      double rear_right_velocity = 0.0;

      if (cmd_twist_)
      {
        // Extract cmd_vel
        double linear_velocity = cmd_twist_->linear.x;
        double angular_z = cmd_twist_->angular.z;

        double curve_radius;
        if (fabs(tan(max_steering_angle_ * angular_z)) < 1e-6)
        {
          curve_radius = std::numeric_limits<double>::infinity();
        }
        else
        {
          curve_radius = wheelbase_distance_ / tan(max_steering_angle_ * angular_z);
        }

        double angular_velocity = linear_velocity / curve_radius;

        if (std::isinf(curve_radius))
        {
          rear_right_velocity = linear_velocity / wheel_radius_;
          rear_left_velocity = -linear_velocity / wheel_radius_;
        }
        else
        {
          rear_right_velocity = angular_velocity * (curve_radius - (kingpin_distance_ / 2)) / wheel_radius_;
          rear_left_velocity = -angular_velocity * (curve_radius + (kingpin_distance_ / 2)) / wheel_radius_;
        }

        front_right_angle = atan(wheelbase_distance_ / (curve_radius - (kingpin_distance_ / 2)));
        front_left_angle = atan(wheelbase_distance_ / (curve_radius + (kingpin_distance_ / 2)));
        // Apply velocities to rear wheels
        joints_rotation_[REAR_LEFT]->SetParam("vel", 0, rear_left_velocity);
        joints_rotation_[REAR_RIGHT]->SetParam("vel", 0, rear_right_velocity);

        // Calculate time step for PID controller
        double dt = (current_time - last_pid_update_time_).Double();
        last_pid_update_time_ = current_time;

        // Apply steering angles using PID controllers
        double current_pos_front_left = joints_rotation_[FRONT_LEFT]->Position(0);
        double front_left_force = calculatePID(pid_controller_front_left_, front_left_angle, current_pos_front_left, dt);
        joints_rotation_[FRONT_LEFT]->SetForce(0, front_left_force);

        double current_pos_front_right = joints_rotation_[FRONT_RIGHT]->Position(0);
        double front_right_force = calculatePID(pid_controller_front_right_, -front_right_angle, current_pos_front_right, dt);
        joints_rotation_[FRONT_RIGHT]->SetForce(0, front_right_force);
      }
    }
  }

  void GazeboRosWheelsSteerable::UpdateOdometryEncoder()
  {
    // Get angular velocities of rear joints
    double vl = (-1) * joints_rotation_[REAR_LEFT]->GetVelocity(0);
    double vr = joints_rotation_[REAR_RIGHT]->GetVelocity(0);

    common::Time current_time = parent_->GetWorld()->SimTime();
    double seconds_since_last_update =
        (current_time - last_odom_update_).Double();
    last_odom_update_ = current_time;

    double b = kingpin_distance_;

    // Book: Sigwart 2011 Autonomous Mobile Robots page:337
    // Calculate left and right wheel distance
    double sl = vl * wheel_radius_ * seconds_since_last_update;
    double sr = vr * wheel_radius_ * seconds_since_last_update;
    double ssum = sl + sr;

    double sdiff = sr - sl;

    double dx = (ssum) / 2.0 * cos(pose_encoder_.theta + (sdiff) / (2.0 * b));
    double dy = (ssum) / 2.0 * sin(pose_encoder_.theta + (sdiff) / (2.0 * b));
    double dtheta = (sdiff) / b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta / seconds_since_last_update;
    double v = sqrt(dx * dx + dy * dy) / seconds_since_last_update;

    tf2::Quaternion qt;
    tf2::Vector3 vt;
    qt.setRPY(0, 0, pose_encoder_.theta);
    vt = tf2::Vector3(pose_encoder_.x, pose_encoder_.y, 0);

    odom_.pose.pose.position.x = vt.x();
    //correct orientation of y coordinate
    odom_.pose.pose.position.y = (-1) * vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    //correct orientation for Z Axis
    odom_.pose.pose.orientation.z = (-1) * qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = v;
    odom_.twist.twist.linear.y = 0;
  }

  void GazeboRosWheelsSteerable::GetGroundTruth()
  {
    // Get data from Gazebo world
    ignition::math::Pose3d pose = parent_->WorldPose();

    tf2::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());

    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();
    odom_.pose.pose.position.z = pose.Pos().Z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    // Get velocity in /odom frame
    ignition::math::Vector3d linear = parent_->WorldLinearVel();
    odom_.twist.twist.angular.z = parent_->WorldAngularVel().Z();

    // Convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.Rot().Yaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
    odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();
  }

  void GazeboRosWheelsSteerable::SetCovariance()
  {
    // Set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 10000000.0;
    odom_.pose.covariance[21] = 10000000.0;
    odom_.pose.covariance[28] = 10000000.0;
    odom_.pose.covariance[35] = 0.001;
  }

  void GazeboRosWheelsSteerable::PublishOdometry()
  {
    rclcpp::Time current_time = ros_node_->get_clock()->now();
    std::string odom_frame = frame_odom_;
    std::string base_footprint_frame = frame_base_;

    // Publish TF
    if (publishOdomTF_)
    {
      geometry_msgs::msg::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = odom_frame;
      odom_trans.child_frame_id = base_footprint_frame;

      odom_trans.transform.translation.x = odom_.pose.pose.position.x;
      odom_trans.transform.translation.y = odom_.pose.pose.position.y;
      odom_trans.transform.translation.z = odom_.pose.pose.position.z;
      odom_trans.transform.rotation = odom_.pose.pose.orientation;

      transform_broadcaster_->sendTransform(odom_trans);
    }

    SetCovariance();

    // Set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_->publish(odom_);

    GetGroundTruth();

    // Set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = frame_ground_truth_;
    odom_.child_frame_id = base_footprint_frame;

    ground_truth_odometry_publisher_->publish(odom_);
  }

  void GazeboRosWheelsSteerable::FiniChild()
  {
    alive_ = false;
  }

  void GazeboRosWheelsSteerable::callbackTopicCMD(const geometry_msgs::msg::Twist::SharedPtr cmd_msg)
  {
    cmd_twist_ = cmd_msg;
  }

  double GazeboRosWheelsSteerable::calculatePID(PID_Controller_State &state, double setValue, double currentValue, double dt)
  {
    // Calculate error
    double error = setValue - currentValue;

    // Proportional term
    double Pout = state.pid_p_ * error;

    // Integral term
    state.integral_ += error * dt;
    double Iout = state.pid_i_ * state.integral_;

    // Derivative term
    double derivative = (error - state.pre_error_) / dt;
    double Dout = state.pid_d_ * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > state.max_effort_pid_)
      output = state.max_effort_pid_;
    else if (output < (-state.max_effort_pid_))
      output = (-state.max_effort_pid_);

    // Save error to previous error
    state.pre_error_ = error;

    return output;
  }

  void GazeboRosWheelsSteerable::setPIDParameters(PID_Controller_State &state, double p, double i, double d, double maxEffort)
  {
    state.max_effort_pid_ = maxEffort;
    state.pid_p_ = p;
    state.pid_i_ = i;
    state.pid_d_ = d;
    state.integral_ = 0;
    state.pre_error_ = 0;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelsSteerable)
} // namespace gazebo
