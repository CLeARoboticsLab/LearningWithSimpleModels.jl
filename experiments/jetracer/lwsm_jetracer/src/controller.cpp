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
    double x_, y_, v_, phi_;
    std::vector<double> spline_coeffs_;
    bool started_;

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
      // TODO return if not started 
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