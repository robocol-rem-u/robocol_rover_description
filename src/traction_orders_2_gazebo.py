#!/usr/bin/env python3
import rospy
from master_msgs.msg import traction_Orders,rpm,sensibility
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

traction_present = traction_Orders()
cmd_vel_msg = Twist()
f_L_wheel = Float64
m_L_wheel = Float64
b_L_wheel = Float64
f_R_wheel = Float64
m_R_wheel = Float64
b_R_wheel = Float64

def node_traction_gazebo():
	global pub_cmd_vel,pub_control,controllers_list
	rospy.init_node('traction_2_gazebo',anonymous=True)
	rospy.Subscriber('topic_traction_orders',traction_Orders,traction_Orders_Callback)
	pub_cmd_vel = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
	controllers_list = ["back_wheel_L_joint_velocity_controller","back_wheel_R_joint_velocity_controller","front_wheel_L_joint_velocity_controller","front_wheel_R_joint_velocity_controller","middle_wheel_L_joint_velocity_controller","middle_wheel_R_joint_velocity_controller"]
	pub_control = {}
	for controller in controllers_list:
		topic_name = "/robocol_rover/"+controller+"/command"
		pub_control[controller] = rospy.Publisher(topic_name,Float64,queue_size=1)
	rate = rospy.Rate (10)
	while not rospy.is_shutdown ():
		rate.sleep()

def traction_Orders_Callback(param):
	global pub_cmd_vel,cmd_vel_msg,pub_control,controllers_list,controllers_list
	global f_L_wheel,m_L_wheel,b_L_wheel,f_R_wheel,m_R_wheel,b_R_wheel
	rpm_l = param.rpm_l/10.0
	rpm_r = param.rpm_r/10.0
	#cmd_vel_msg.linear.x = rpm_l
	#cmd_vel_msg.linear.y = rpm_r
	#pub_cmd_vel.publish(cmd_vel_msg)
	f_L_wheel.data = rpm_l
	m_L_wheel.data = rpm_l
	b_L_wheel.data = rpm_l
	f_R_wheel.data = rpm_r
	m_R_wheel.data = rpm_r
	b_R_wheel.data = rpm_r
	print(pub_control[controllers_list[0]])
	pub_control[controllers_list[0]].publish(f_L_wheel)
	pub_control[controllers_list[1]].publish(m_L_wheel)
	pub_control[controllers_list[2]].publish(b_L_wheel)
	pub_control[controllers_list[3]].publish(f_R_wheel)
	pub_control[controllers_list[4]].publish(m_R_wheel)
	pub_control[controllers_list[5]].publish(b_R_wheel)
	print('L: ',rpm_l,'R: ',rpm_r)

if __name__ == '__main__':
	try:
		node_traction_gazebo()
	except rospy.ROSInterruptException:
		pass
