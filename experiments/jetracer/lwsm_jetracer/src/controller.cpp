#include <iostream>
#include <signal.h>
#include <cmath>

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>

// Flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Stop command
const std::vector<double> g_stop{0.0, 0.0};

// Handles ctrl+c of the node
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

class Controller
{
  public:
    Controller() : started_(false), kx_(0.0), ky_(0.0), kv_(0.0), kphi_(0.0),
        spline_coeffs_(8, 0.0)
    {
      ROS_INFO_STREAM("Starting controller");

      const auto queue_size = 100;
      throttle_pub_ = nh_.advertise<std_msgs::Float32>("jetracer/throttle", queue_size);
      steering_pub_ = nh_.advertise<std_msgs::Float32>("jetracer/steering", queue_size);

      start_time_sub_ = nh_.subscribe("start_time", queue_size, &Controller::startTimeCallback, this);
      spline_gains_sub_ = nh_.subscribe("jetracer/spline_gains", queue_size, &Controller::splineGainsCallback, this);
      pose_sub_ = nh_.subscribe("jetracer/pose", queue_size, &Controller::poseCallback, this);
      twist_sub_ = nh_.subscribe("jetracer/twist", queue_size, &Controller::twistCallback, this);
    }

    void shutdown()
    {
      ROS_INFO_STREAM("Stopping and exiting..." );
      ros::shutdown();
      std::cout << "Exited." << std::endl;
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher throttle_pub_;
    ros::Publisher steering_pub_;
    ros::Subscriber start_time_sub_;
    ros::Subscriber spline_gains_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;

    ros::Time start_time_;
    double t_;
    double kx_, ky_, kv_, kphi_;
    double x_, y_, v_, phi_;
    std::vector<double> spline_coeffs_;
    bool started_;
    std::vector<int> seg_idxs_;
    std::vector<double> ts_;
    std::vector<std::vector<double>> xs_;
    std::vector<std::vector<double>> us_;

    static constexpr double PI = 3.14159265358979323846264;

    void startTimeCallback(std_msgs::Time time)
    {
      seg_idxs_.clear();
      ts_.clear();
      xs_.clear();
      us_.clear();
      
      start_time_ = time.data;
      started_ = !start_time_.is_zero();
      if (started_)
        ROS_INFO_STREAM("Starting control");
      else
      {
        ROS_INFO_STREAM("Stopping robot");
        publishCommand(g_stop);
      }
    }

    void splineGainsCallback(std_msgs::Float64MultiArray spline_gains)
    {
      for(int i = 0; i < 8; i++) 
        spline_coeffs_[i] = spline_gains.data[i];
      kx_ = spline_gains.data[8];
      ky_ = spline_gains.data[9];
      kv_ = spline_gains.data[10];
      kphi_ = spline_gains.data[11];
      seg_idxs_.push_back(ts_.size());
      ROS_INFO_STREAM("New spline and gains loaded");
    }

    void poseCallback(geometry_msgs::PoseStamped pose)
    {
      x_ = pose.pose.position.x;
      y_ = pose.pose.position.y;
      phi_ = headingAngle(pose);
      control();
    }

    void twistCallback(geometry_msgs::TwistStamped twist)
    {
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

    void control()
    {
      if (!started_)
        return;

      std::vector<double> setpoint = evaluateSpline();
      double x_des = setpoint[0];
      double y_des = setpoint[1];
      double xdot_des = setpoint[2];
      double ydot_des = setpoint[3];

      double xdot_tilde_des = xdot_des + kx_*(x_des - x_);
      double ydot_tilde_des = ydot_des + ky_*(y_des - y_);
      double v_des = sqrt(pow(xdot_tilde_des,2.0) + pow(ydot_tilde_des,2.0));

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
      
      double throttle = clamp(kv_*(v_des - v_), 0.0, 1.0);
      double steering = clamp(kphi_*(phi_des - phi_), -1.0, 1.0);
      std::vector<double> command{throttle, steering};
      publishCommand(command);

      std::vector<double> x{x_, y_, v_, phi_};
      ts_.push_back(t_);
      xs_.push_back(x);
      us_.push_back(command);    
    }

    template <typename T>
    T clamp(const T& n, const T& lower, const T& upper) {
      return std::max(lower, std::min(n, upper));
    }

    std::vector<double> evaluateSpline()
    {
      ros::Duration d = ros::Time::now() - start_time_;
      t_ = d.toSec();

      double x = spline_coeffs_[0]*pow(t_, 3.0) + spline_coeffs_[1]*pow(t_, 2.0)
                  + spline_coeffs_[2]*t_ + spline_coeffs_[3];
      double y = spline_coeffs_[4]*pow(t_, 3.0) + spline_coeffs_[5]*pow(t_, 2.0)
                  + spline_coeffs_[6]*t_ + spline_coeffs_[7];
      double xdot = 3.0*spline_coeffs_[0]*pow(t_, 2.0) + 2.0*spline_coeffs_[1]*t_
                  + spline_coeffs_[2];
      double ydot = 3.0*spline_coeffs_[4]*pow(t_, 2.0) + 2.0*spline_coeffs_[5]*t_
                  + spline_coeffs_[6];

      std::vector<double> setpoint{x, y, xdot, ydot};
      return setpoint;
    }

    void publishCommand(std::vector<double> command)
    {
      std_msgs::Float32 t, s;
      t.data = command[0];
      s.data = command[1];
      throttle_pub_.publish(t);
      steering_pub_.publish(s);
    }
};

auto main(int argc, char **argv) -> int
{
  // initialize node
  ros::init(argc, argv, "controller", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  Controller controller;

  while (!g_request_shutdown && ros::ok())
  {
    ros::spinOnce();
  }

  // shutdown gracefully if node is interrupted
  controller.shutdown();

  return 0;
}