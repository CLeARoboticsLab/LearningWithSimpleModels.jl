#include <iostream>
#include <signal.h>

#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

// Flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Handles ctrl+c of the node
void sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

auto main(int argc, char **argv) -> int
{
  ros::init(argc, argv, "test_qual_control", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);

  unitree_legged_msgs::HighCmd high_cmd_ros;

  high_cmd_ros.head[0] = 0xFE;
  high_cmd_ros.head[1] = 0xEF;
  high_cmd_ros.levelFlag = HIGHLEVEL;
  high_cmd_ros.mode = 0;  // 0: idle, 2: walk
  high_cmd_ros.gaitType = 0;  // 0: idle, 1: trot, 2: trot running, 3: climb stair, 4: trot obstacle
  high_cmd_ros.speedLevel = 0; // not used (only for mode=3) 
  high_cmd_ros.footRaiseHeight = 0; // foot height while walking, delta value
  high_cmd_ros.bodyHeight = 0; // don't change
  high_cmd_ros.euler[0] = 0; // euler angles are for mode=1 only
  high_cmd_ros.euler[1] = 0;
  high_cmd_ros.euler[2] = 0;
  high_cmd_ros.velocity[0] = 0.0f; // forward speed, -1 ~ +1 m/s
  high_cmd_ros.velocity[1] = 0.0f;  // side speed
  high_cmd_ros.yawSpeed = 0.0f; // rotation speed, rad/s
  high_cmd_ros.reserve = 0;

  // start up in idle
  pub.publish(high_cmd_ros);

  ROS_INFO_STREAM("Robot walking...");
  while (!g_request_shutdown && ros::ok())
  {
    high_cmd_ros.mode = 2;
    high_cmd_ros.gaitType = 1;
    high_cmd_ros.velocity[0] = 0.1f; 
    high_cmd_ros.yawSpeed = 0.4f;

    pub.publish(high_cmd_ros);

    ros::spinOnce();
    rate.sleep();
  }

  // shutdown gracefully if node is interrupted
  high_cmd_ros.mode = 0;
  high_cmd_ros.gaitType = 0;
  high_cmd_ros.velocity[0] = 0.0f; 
  high_cmd_ros.yawSpeed = 0.0f;
  pub.publish(high_cmd_ros);
  ROS_INFO_STREAM("Robot stopped");
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("Stopping and exiting..." );
  ros::shutdown();
  std::cout << "Exited." << std::endl;

  return 0;
}