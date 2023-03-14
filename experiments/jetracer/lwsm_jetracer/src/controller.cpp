#include <iostream>
#include <signal.h>

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Time.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

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

    Controller()
    {
      ROS_INFO_STREAM("Starting controller");

      const auto queue_size = 100;
      throttle_pub_ = nh_.advertise<std_msgs::Float32>("jetracer/throttle", queue_size);
      steering_pub_ = nh_.advertise<std_msgs::Float32>("jetracer/steering", queue_size);

      start_time_sub_ = nh_.subscribe("start_time", queue_size, &Controller::start_time_callback, this);
      spline_sub_ = nh_.subscribe("jetracer/spline", queue_size, &Controller::spline_callback, this);
      pose_sub_ = nh_.subscribe("jetracer/pose", queue_size, &Controller::pose_callback, this);
      twist_sub_ = nh_.subscribe("jetracer/twist", queue_size, &Controller::twist_callback, this);
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

    void start_time_callback(std_msgs::Time time)
    {
      // TODO
    }

    void spline_callback(std_msgs::Float64MultiArray spline)
    {
      // TODO
    }

    void pose_callback(geometry_msgs::PoseStamped pose)
    {
      // TODO
    }

    void twist_callback(geometry_msgs::TwistStamped twist)
    {
      // TODO
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

  }

  // shutdown gracefully if node is interrupted
  controller.shutdown();

  return 0;
}