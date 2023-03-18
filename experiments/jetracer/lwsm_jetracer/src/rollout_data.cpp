#include <string>

#include <ros/ros.h>

#include "lwsm_jetracer/rollout_data_server.hpp"
#include "lwsm_jetracer/RolloutData.h"

class RolloutData
{
  public:
    RolloutData()
    {
      int port;
      nh_.getParam("rollout_data/port", port);

      ROS_INFO_STREAM("Starting rollout data server on port " << port);
      rollout_data_server_ = new communication::RolloutDataServer(port);

      sub_ = nh_.subscribe("jetracer/rollout_data", 100, &RolloutData::sub_callback, this);
    }

    ~RolloutData()
    {
      delete rollout_data_server_;
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    communication::RolloutDataServer *rollout_data_server_;

    void sub_callback(lwsm_jetracer::RolloutData data)
    {
      rollout_data_server_->update(data);
    }

};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "rollout_data");

  RolloutData rollout_data;

  ros::spin();
  
  return 0;
}