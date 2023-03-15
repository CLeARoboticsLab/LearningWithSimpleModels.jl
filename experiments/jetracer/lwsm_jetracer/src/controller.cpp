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
    Controller() : started_(false)
    {
      ROS_INFO_STREAM("Starting controller");

      nh_.getParam("controller/kx", kx_);
      nh_.getParam("controller/ky", ky_);
      nh_.getParam("controller/kv", kv_);
      nh_.getParam("controller/kphi", kphi_);

      const auto queue_size = 100;
      throttle_pub_ = nh_.advertise<std_msgs::Float32>("jetracer/throttle", queue_size);
      steering_pub_ = nh_.advertise<std_msgs::Float32>("jetracer/steering", queue_size);

      start_time_sub_ = nh_.subscribe("start_time", queue_size, &Controller::startTimeCallback, this);
      spline_sub_ = nh_.subscribe("jetracer/spline", queue_size, &Controller::splineCallback, this);
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
    ros::Subscriber spline_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber twist_sub_;

    ros::Time start_time_;
    double kx_, ky_, kv_, kphi_;
    double x_, y_, v_, phi_;
    std::vector<double> spline_coeffs_;
    bool started_;

    static constexpr double PI = 3.14159265358979323846264;

    void startTimeCallback(std_msgs::Time time)
    {
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

    void splineCallback(std_msgs::Float64MultiArray spline)
    {
      spline_coeffs_ = spline.data;
      ROS_INFO_STREAM("New spline loaded");
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
      // TODO add sigmoid instead of clamping?
    }

    template <typename T>
    T clamp(const T& n, const T& lower, const T& upper) {
      return std::max(lower, std::min(n, upper));
    }

    std::vector<double> evaluateSpline()
    {
      ros::Duration d = ros::Time::now() - start_time_;
      double t = d.toSec();

      double x = spline_coeffs_[0]*pow(t, 3.0) + spline_coeffs_[1]*pow(t, 2.0)
                  + spline_coeffs_[2]*t + spline_coeffs_[3];
      double y = spline_coeffs_[4]*pow(t, 3.0) + spline_coeffs_[5]*pow(t, 2.0)
                  + spline_coeffs_[6]*t + spline_coeffs_[7];
      double xdot = 3.0*spline_coeffs_[0]*pow(t, 2.0) + 2.0*spline_coeffs_[1]*t
                  + spline_coeffs_[2];
      double ydot = 3.0*spline_coeffs_[4]*pow(t, 2.0) + 2.0*spline_coeffs_[5]*t
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