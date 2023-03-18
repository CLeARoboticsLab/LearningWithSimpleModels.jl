#!/usr/bin/env python3

import rospy
from jetracer.nvidia_racecar import NvidiaRacecar
from std_msgs.msg import Float32

class ServoController:

	def __init__(self):
        
		# do not adjust these gains/offsets, instead adjust the ones below
		self.car = NvidiaRacecar()
		self.car.steering_gain = 1.0
		self.car.steering_offset = 0.0
		self.car.throttle_gain = 1

		# these are adjustable
		self.sgain = -1.0
		self.soffset = -0.4
		self.tgain = -0.5
		self.toffset = 0.05

		self.command_steering(0.0)
		self.command_throttle(0.0)
		self.throt_sub = rospy.Subscriber("jetracer/throttle", Float32, self.throttle_callback)
		self.steer_sub = rospy.Subscriber("jetracer/steering", Float32, self.steering_callback)

	def throttle_callback(self, throt):
		self.command_throttle(throt.data)

	def steering_callback(self, steer):
		self.command_steering(steer.data)

	def command_throttle(self, throt_in):
		throt_out = self.tgain * throt_in + self.toffset			
		self.car.throttle = self.clamp(throt_out, -1.0, 1.0)

	def command_steering(self, steer_in):
		steer_out = self.sgain * steer_in + self.soffset		
		self.car.steering = self.clamp(steer_out, -1.0, 1.0)		

	def clamp(self, num, min_value, max_value):
		return max(min(num, max_value), min_value)

	def shutdown(self):
		self.command_steering(0.0)
		self.command_throttle(0.0)
		print("Stopping JetRacer")

if __name__ == '__main__':
	print("Starting JetRacer servo controller")
	rospy.init_node('servo_controller', anonymous=True)
	s = ServoController()
	rospy.on_shutdown(s.shutdown)
	rospy.spin()
