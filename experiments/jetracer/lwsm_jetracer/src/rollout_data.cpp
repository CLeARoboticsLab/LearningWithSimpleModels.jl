#include <string>

#include <ros/ros.h>

#include "lwsm_jetracer/rollout_data_server.hpp"

#include <ros_sockets/server.hpp>

class RolloutData
{
  public:

    RolloutData()
    {

    }

    ~RolloutData()
    {

    }

  private:

    // ROS objects
    ros::NodeHandle nh_;

};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "rollout_data");

  RolloutData rollout_data;

  ros::spin();
  
  return 0;
}