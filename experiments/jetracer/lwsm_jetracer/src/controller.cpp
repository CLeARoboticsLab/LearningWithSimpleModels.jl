#include <iostream>
#include <signal.h>

#include <ros/ros.h>

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

    }

    void shutdown()
    {
      ROS_INFO_STREAM("Stopping and exiting..." );
      ros::shutdown();
      std::cout << "Exited." << std::endl;
    }

  private:

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