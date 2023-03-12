#!/usr/bin/env python3

import rospy
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32

car = NvidiaRacecar()
car.steering_gain = 0.65
car.steering_offset = -0.16
car.throttle_gain = 1
car.steering = 0.0
car.throttle = 0.0

def throttle_callback(throt):
    car.throttle = throt.data
    rospy.loginfo("Throttle: %s", str(throt.data))

def steering_callback(steer):
    car.steering = steer.data
    rospy.loginfo("Steering: %s", str(steer.data))

def shutdown_hook():
    car.throttle = 0.0
    car.steering = 0.0
    print("Stopping JetRacer")

def run():
    rospy.on_shutdown(shutdown_hook)
    rospy.init_node('jetracer', anonymous=True)
    rospy.Subscriber("jetracer/throttle", Float32, throttle_callback)
    rospy.Subscriber("jetracer/steering", Float32, steering_callback)

    rospy.spin()

if __name__ == '__main__':
    print("Starting JetRacer servo controller")
    run()