#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
#from geometry_msgs.msg import Twist

model = ModelState()
#move = Twist()

### NODO PRINCIPAL ###
def node_gazebo():
	global model, move
	#creacion del nodo
	rospy.init_node('node_gazebo_rover',anonymous=True)
	#se subscribe al topico traction orders
	#rospy.Subscriber ('topic_traction_orders', traction_Orders, traction_Orders_Callback)
	# publica en RPM, Current y POTS
	pub_gazebo = rospy.Publisher('/gazebo/set_model_state',ModelState, queue_size=10)
	pub_gazebo = rospy.Publisher('/gazebo/set_link_state',ModelState, queue_size=10)
	rate = rospy.Rate (10)
	k = 0
	while not rospy.is_shutdown ():
		if k == 2:
			model.model_name = 'robocol_rover2'
			model.pose.position.x = 0
			model.pose.position.y = 0
			model.pose.position.z = 1
			model.pose.orientation.x = 0
			model.pose.orientation.y = 0
			model.pose.orientation.z = 0
			model.pose.orientation.w = 0
			pub_gazebo.publish(model)
			model = ModelState()
		if k > 10:
			model.model_name = 'robocol_rover2'
			#model.twist.linear.x = 0
			#model.twist.linear.y = 0.1
			#model.twist.linear.z = 0
			#model.twist.angular.x = 0
			#model.twist.angular.y = 0
			#model.twist.angular.z = 0
			pub_gazebo.publish(model)
		#print('s')
		k = k + 1
		rate.sleep()


if __name__ == '__main__':
	try:
		node_gazebo()
	except rospy.ROSInterruptException:
		pass
