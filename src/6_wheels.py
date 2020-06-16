#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class RobocolRover(object):
	def __init__(self):
		rospy.loginfo("Rover Initialising...")
		# # TODO: Ackerman stuff
		self.distance_axis = 0.3
		self.distance_front_center = 0.5
		self.distance_back_center = 0.5
		self.publishers_curiosity_d = {}
		self.controller_ns = "robocol_rover"
		self.controller_command = "command"
		self.controllers_list = [   "back_wheel_L_joint_velocity_controller",
									"back_wheel_R_joint_velocity_controller",
									"front_wheel_L_joint_velocity_controller",
									"front_wheel_R_joint_velocity_controller",
									"middle_wheel_L_joint_velocity_controller",
									"middle_wheel_R_joint_velocity_controller"
								]
		for controller_name in self.controllers_list:
			topic_name = "/"+self.controller_ns+"/"+controller_name+"/"+self.controller_command
			self.publishers_curiosity_d[controller_name] = rospy.Publisher(topic_name,Float64,queue_size=1)
		self.wait_publishers_to_be_ready()
		self.init_publisher_variables()
		self.init_state()
		self.cmd_vel_msg = Twist()
		cmd_vel_topic = "/cmd_vel"
		rospy.Subscriber(cmd_vel_topic, Twist, self.cmd_vel_callback)
		rospy.logwarn("RobocolRover...READY")
	def cmd_vel_callback(self, msg):
		self.cmd_vel_msg = msg
	def wait_publishers_to_be_ready(self):
		rate_wait = rospy.Rate(10)
		for controller_name, publisher_obj in self.publishers_curiosity_d.iteritems():
			publisher_ready = False
			while not publisher_ready:
				rospy.logwarn("Checking Publisher for ==>"+str(controller_name))
				pub_num = publisher_obj.get_num_connections()
				publisher_ready = (pub_num > 0)
				rate_wait.sleep()
			rospy.loginfo("Publisher ==>" + str(controller_name) + "...READY")
	def init_publisher_variables(self):
		"""
		We create variables for more pythonic access access to publishers
		and not need to access any more
		:return:
		"""
		# Get the publishers for wheel speed
		self.back_wheel_L = self.publishers_curiosity_d[self.controllers_list[0]]
		self.back_wheel_R = self.publishers_curiosity_d[self.controllers_list[1]]
		self.front_wheel_L = self.publishers_curiosity_d[self.controllers_list[2]]
		self.front_wheel_R = self.publishers_curiosity_d[self.controllers_list[3]]
		self.middle_wheel_L = self.publishers_curiosity_d[self.controllers_list[4]]
		self.middle_wheel_R = self.publishers_curiosity_d[self.controllers_list[5]]
		# Init Messages
		self.back_wheel_L_velocity_msg = Float64()
		self.back_wheel_R_velocity_msg = Float64()
		self.front_wheel_L_velocity_msg = Float64()
		self.front_wheel_R_velocity_msg = Float64()
		self.middle_wheel_L_velocity_msg = Float64()
		self.middle_wheel_R_velocity_msg = Float64()
	def init_state(self):
		self.set_wheels_speed(0.0,0.0)
	def set_wheels_speed(self,turning_speed_left,turning_speed_right):
		"""
		Sets the turning speed in radians per second
		:param turning_speed: In radians per second
		:return:
		"""
		# TODO: turning_speed for each wheel should change based on ackerman.
		self.front_wheel_L_velocity_msg.data = -1*turning_speed_left
		self.middle_wheel_L_velocity_msg.data = -1*turning_speed_left
		self.back_wheel_L_velocity_msg.data = turning_speed_left
		self.front_wheel_R_velocity_msg.data = turning_speed_right
		self.middle_wheel_R_velocity_msg.data = turning_speed_right
		self.back_wheel_R_velocity_msg.data = turning_speed_right
		self.front_wheel_L.publish(self.front_wheel_L_velocity_msg)
		self.middle_wheel_L.publish(self.middle_wheel_L_velocity_msg)
		self.back_wheel_L.publish(self.back_wheel_L_velocity_msg)
		self.front_wheel_R.publish(self.front_wheel_R_velocity_msg)
		self.middle_wheel_R.publish(self.middle_wheel_R_velocity_msg)
		self.back_wheel_R.publish(self.back_wheel_R_velocity_msg)
	def move_forwards(self):
		self.set_wheels_speed(10.0,10.0)
	def move_backwards(self):
		self.set_wheels_speed(-10.0,-10.0)
	def move_turn_left(self):
		self.set_wheels_speed(10.0,0.0)
	def move_turn_right(self):
		self.set_wheels_speed(0.0,10.0)
	def move_turn_stop(self):
		self.set_wheels_speed(0.0,0.0)
	def move_with_cmd_vel(self):
		wheel_speed_left = self.cmd_vel_msg.linear.x
		wheel_speed_right = self.cmd_vel_msg.linear.y
		self.set_wheels_speed(-wheel_speed_left,-wheel_speed_right)
if __name__ == "__main__":
	rospy.init_node("Robocol_Rover_node", log_level=rospy.INFO)
	robocol_rover_object = RobocolRover()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		robocol_rover_object.move_with_cmd_vel()
		rate.sleep()
