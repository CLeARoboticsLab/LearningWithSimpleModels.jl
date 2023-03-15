#!/usr/bin/env python3

import rospy
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32

class ServoController:

    def __init__(self):
        self.car = NvidiaRacecar()
        self.car.steering_gain = 0.65
        self.car.steering_offset = -0.16
        self.car.throttle_gain = 1
        self.car.steering = 0.0
        self.car.throttle = 0.0
        self.throt_sub = rospy.Subscriber("jetracer/throttle", Float32, self.throttle_callback)
        self.steer_sub = rospy.Subscriber("jetracer/steering", Float32, self.steering_callback)

    def throttle_callback(self, throt):
        self.car.throttle = throt.data
        rospy.loginfo("Throttle: %s", str(throt.data))

    def steering_callback(self, steer):
        self.car.steering = steer.data
        rospy.loginfo("Steering: %s", str(steer.data))

    def shutdown(self):
        self.car.throttle = 0.0
        self.car.steering = 0.0
        print("Stopping JetRacer")

if __name__ == '__main__':
    print("Starting JetRacer servo controller")
    rospy.init_node('servo_controller', anonymous=True)
    s = ServoController()
    rospy.on_shutdown(s.shutdown)
    rospy.spin()