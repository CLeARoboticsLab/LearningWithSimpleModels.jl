#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class ServoControllerSimulation:

    def __init__(self):
        self.cmd = Twist()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.throt_sub = rospy.Subscriber("jetracer/throttle", Float32, self.throttle_callback)
        self.steer_sub = rospy.Subscriber("jetracer/steering", Float32, self.steering_callback)

    def throttle_callback(self, throt):
        self.cmd.linear.x = throt.data
        self.pub.publish(self.cmd)

    def steering_callback(self, steer):
        self.cmd.angular.z = steer.data
        self.pub.publish(self.cmd)

    def shutdown(self):
        print("Stopping")
        self.pub.publish(Twist())

if __name__ == '__main__':
    rospy.init_node('servo_controller_sim', anonymous=True)
    s = ServoControllerSimulation()
    rospy.on_shutdown(s.shutdown)
    rospy.spin()