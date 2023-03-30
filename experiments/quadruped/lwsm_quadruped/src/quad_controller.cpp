#include <iostream>
#include <signal.h>
#include <cmath>

#include <ros/ros.h>
#include "std_msgs/Time.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>
#include <unitree_legged_msgs/HighCmd.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

#include "lwsm_jetracer/RolloutData.h"

using namespace UNITREE_LEGGED_SDK;

// Flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Handles ctrl+c of the node
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

class Controller
{
  public:
    Controller() : started_(false), control_lock_(false),
        kx_(0.0), ky_(0.0), kv_(0.0), kphi_(0.0), ka_(0.0), komg_(0.0), spline_coeffs_(8, 0.0)
    {
      ROS_INFO_STREAM("Starting controller");

      cmd_.head[0] = 0xFE;
      cmd_.head[1] = 0xEF;
      cmd_.levelFlag = HIGHLEVEL;
      cmd_.mode = 0;            // 0: idle, 2: walk
      cmd_.gaitType = 0;        // 0: idle, 1: trot, 2: trot running, 3: climb stair, 4: trot obstacle
      cmd_.speedLevel = 0;      // not used (only for mode=3) 
      cmd_.footRaiseHeight = 0; // foot height while walking, delta value
      cmd_.bodyHeight = 0;      // don't change
      cmd_.euler[0] = 0;        // euler angles are for mode=1 only
      cmd_.euler[1] = 0;
      cmd_.euler[2] = 0;
      cmd_.velocity[0] = 0.0f;  // forward speed, -1 ~ +1 m/s
      cmd_.velocity[1] = 0.0f;  // side speed
      cmd_.yawSpeed = 0.0f;     // rotation speed, rad/s
      cmd_.reserve = 0;

      nh_.getParam("quad_controller/cycle_rate", cycle_rate_);

      const auto queue_size = 1000;
      cmd_pub_ = nh_.advertise<unitree_legged_msgs::HighCmd>("high_cmd", queue_size);
      //TODO topic name
      data_pub_ = nh_.advertise<lwsm_jetracer::RolloutData>("jetracer/rollout_data", queue_size);

      start_time_sub_ = nh_.subscribe("start_time", queue_size, &Controller::startTimeCallback, this);
      //TODO topic name for spline_gains
      spline_gains_sub_ = nh_.subscribe("quad/spline_gains", queue_size, &Controller::splineGainsCallback, this);
      pose_sub_ = nh_.subscribe("vrpn_client_node/quad/pose", queue_size, &Controller::poseCallback, this);
      twist_sub_ = nh_.subscribe("vrpn_client_node/quad/twist", queue_size, &Controller::twistCallback, this);
      stop_robot();
    }

    double cycle_rate(){return cycle_rate_;}

    void shutdown()
    {
      stop_robot();
      ros::Duration(1.0).sleep();
      ROS_INFO_STREAM("Stopping and exiting..." );
      ros::shutdown();
      std::cout << "Exited." << std::endl;
    }

    void control()
    {
      if (!started_)
        return;

      std::vector<double> setpoint = evaluateSpline();
      double x_des = setpoint[0];
      double y_des = setpoint[1];
      double xdot_des = setpoint[2];
      double ydot_des = setpoint[3];
      double xddot_des = setpoint[4];
      double yddot_des = setpoint[5];

      control_lock_ = true;

      double xdot_tilde_des = xdot_des + kx_*(x_des - x_);
      double ydot_tilde_des = ydot_des + ky_*(y_des - y_);
      double v_des = sqrt(pow(xdot_tilde_des,2.0) + pow(ydot_tilde_des,2.0));
      v_des = clamp(v_des, -1.0, 1.0);

      double phi_des;
      if (xdot_tilde_des == 0)
        phi_des = copysign(1.0, ydot_tilde_des)*PI/2;
      else
        phi_des = atan(ydot_tilde_des/xdot_tilde_des);

      if (xdot_tilde_des < 0.0)
        phi_des += PI;
      else if (ydot_tilde_des < 0.0)
        phi_des += 2*PI;

      if (abs(phi_des - phi_) > abs(phi_des - 2*PI - phi_))
        phi_des -= 2*PI;
      else if (abs(phi_des - phi_) > abs(phi_des + 2*PI - phi_))
        phi_des += 2*PI;
      
      // feedforward control inputs
      double v_des_og = sqrt(xdot_des*xdot_des + ydot_des*ydot_des);
      double phi_des_og = atan2(ydot_des, xdot_des);
      double omg_des = -xddot_des/v_des_og*sin(phi_des_og) + yddot_des/v_des_og*cos(phi_des_og);

      double steering = clamp(komg_*omg_des + kphi_*(phi_des - phi_), -1.0, 1.0);

      std::vector<double> command{v_des, steering};
      
      std::vector<double> x{x_, y_, v_, phi_};
      ts_.push_back(t_);
      xs_.push_back(x);
      us_.push_back(command);
      ctrl_setpoints_.push_back(setpoint);

      control_lock_ = false;
      publishCommand(command);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher data_pub_;
    ros::Subscriber start_time_sub_;
    ros::Subscriber spline_gains_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;

    double cycle_rate_;
    ros::Time start_time_;
    double t_;
    double kx_, ky_, kv_, kphi_, ka_, komg_;
    double x_, y_, v_, phi_;
    std::vector<double> spline_coeffs_;
    bool started_, control_lock_;
    std::vector<int> seg_idxs_;
    std::vector<double> ts_;
    std::vector<std::vector<double>> xs_, us_, ctrl_setpoints_;
    unitree_legged_msgs::HighCmd cmd_;

    static constexpr double PI = 3.14159265358979323846264;

    void startTimeCallback(std_msgs::Time time)
    {
      bool started = !time.data.is_zero();
      if (started)
      {
        start_time_ = time.data;
        ROS_INFO_STREAM("Starting control");
        seg_idxs_.clear();
        ts_.clear();
        xs_.clear();
        us_.clear();
        ctrl_setpoints_.clear();
        start_robot();
        started_ = started;
      }
      else
      {
        stop_robot();
        publishRolloutData();
        ROS_INFO_STREAM("Rollout data published");
      }
    }

    void start_robot()
    {
      cmd_.mode = 2;
      cmd_.gaitType = 1;
      cmd_.velocity[0] = 0.0f; 
      cmd_.yawSpeed = 0.0f;
      cmd_pub_.publish(cmd_);
      ROS_INFO_STREAM("Robot started");
    }

    void stop_robot()
    {
      cmd_.mode = 0;
      cmd_.gaitType = 0;
      cmd_.velocity[0] = 0.0f; 
      cmd_.yawSpeed = 0.0f;
      cmd_pub_.publish(cmd_);
      ROS_INFO_STREAM("Robot stopped");
    }

    void splineGainsCallback(std_msgs::Float64MultiArray spline_gains)
    {
      if (control_lock_)
        ROS_WARN_STREAM("Spline/gains loaded in the middle of control. Possible mismatch may follow.");
      for(int i = 0; i < 6; i++) 
        spline_coeffs_[i] = spline_gains.data[i];
      kx_ = spline_gains.data[6];
      ky_ = spline_gains.data[7];
      kv_ = spline_gains.data[8];
      kphi_ = spline_gains.data[9];
      ka_ = spline_gains.data[10];
      komg_ = spline_gains.data[11];
      seg_idxs_.push_back(ts_.size());
      ROS_INFO_STREAM("New spline and gains loaded");
    }

    void poseCallback(geometry_msgs::PoseStamped pose)
    {
      if (control_lock_)
        return;
      x_ = pose.pose.position.x;
      y_ = pose.pose.position.y;
      phi_ = headingAngle(pose);
    }

    void twistCallback(geometry_msgs::TwistStamped twist)
    {
      if (control_lock_)
        return;
      double xdot = twist.twist.linear.x;
      double ydot = twist.twist.linear.y;
      v_ = sqrt(pow(xdot, 2.0) + pow(ydot, 2.0));
    }

    double headingAngle(geometry_msgs::PoseStamped pose)
    {
      tf::Quaternion q(pose.pose.orientation.x, 
                       pose.pose.orientation.y, 
                       pose.pose.orientation.z,
                       pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      return yaw;
    }

    double clamp(double n, double lower, double upper) {
      return std::max(lower, std::min(n, upper));
    }

    std::vector<double> evaluateSpline()
    {
      ros::Duration d = ros::Time::now() - start_time_;
      t_ = d.toSec();

      double x = spline_coeffs_[0]*pow(t_, 2.0) + spline_coeffs_[1]*t_ + spline_coeffs_[2];
      double y = spline_coeffs_[3]*pow(t_, 2.0) + spline_coeffs_[4]*t_ + spline_coeffs_[5];
      double xdot = 2*spline_coeffs_[0]*pow(t_, 1.0) + spline_coeffs_[1];
      double ydot = 2*spline_coeffs_[3]*pow(t_, 1.0) + spline_coeffs_[4];
      double xddot = 2*spline_coeffs_[0];
      double yddot = 2*spline_coeffs_[3];

      std::vector<double> setpoint{x, y, xdot, ydot, xddot, yddot};
      return setpoint;
    }

    void publishCommand(std::vector<double> command)
    {
      cmd_.velocity[0] = (float) command[0];
      cmd_.yawSpeed = (float) command[1];
      cmd_pub_.publish(cmd_);
    }

    void publishRolloutData()
    {
      lwsm_jetracer::RolloutData msg;
      msg.seg_idxs = seg_idxs_;
      msg.ts = ts_;
      std::vector<std_msgs::Float64MultiArray> xs;
      std::vector<std_msgs::Float64MultiArray> us;
      std::vector<std_msgs::Float64MultiArray> ctrl_setpoints;
      for (int i = 0; i < ts_.size(); i++)
      {
        std_msgs::Float64MultiArray x_msg;
        x_msg.data = xs_[i];
        xs.push_back(x_msg);

        std_msgs::Float64MultiArray u_msg;
        u_msg.data = us_[i];
        us.push_back(u_msg);

        std_msgs::Float64MultiArray ctrl_setpoint_msg;
        ctrl_setpoint_msg.data = ctrl_setpoints_[i];
        ctrl_setpoints.push_back(ctrl_setpoint_msg);
      }
      msg.xs = xs;
      msg.us = us;
      msg.ctrl_setpoints = ctrl_setpoints;
      data_pub_.publish(msg);
    }
};

auto main(int argc, char **argv) -> int
{
  // initialize node
  ros::init(argc, argv, "controller", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  Controller controller;

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Rate rate(controller.cycle_rate());

  while (!g_request_shutdown && ros::ok())
  {
    controller.control();
    ros::spinOnce();
    rate.sleep();
  }

  // shutdown gracefully if node is interrupted
  controller.shutdown();

  return 0;
}
