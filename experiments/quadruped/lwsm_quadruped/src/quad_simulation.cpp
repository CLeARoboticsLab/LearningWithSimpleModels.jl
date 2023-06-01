#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include <unitree_legged_msgs/HighCmd.h>

class QuadSimulation
{
  public:
    QuadSimulation()
    {
      const auto queue_size = 1000;
      cmd_sub_ = nh_.subscribe("high_cmd", queue_size, &QuadSimulation::cmdCallback, this);
      cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", queue_size);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Subscriber cmd_sub_;

    void cmdCallback(unitree_legged_msgs::HighCmd cmd)
    {
      geometry_msgs::Twist t;
      t.linear.x = (double) cmd.velocity[0];
      t.angular.z = (double) cmd.yawSpeed;
      cmd_pub_.publish(t);
    }
};

auto main(int argc, char **argv) -> int
{
  ros::init(argc, argv, "quad_simulation");
  QuadSimulation q;
  ros::spin();
  return 0;
}
