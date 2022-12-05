#########

import math
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Twist
import rospy
import tf2_ros
import sys
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
position = Pose()
my_dict = {}

def callback(data):
	global my_dict
	my_dict = {}
	for blocks in range(2601):
		my_dict[int(data.points[blocks].x * (51/20)) ,int( data.points[blocks].y * (51/20))] = data.colors[blocks].r


	#print(data.pose)

def callback1(data):
	print(data.linear.x)
	
def Position(odom):
	global position
	#global yaw	
	position = odom.pose.pose
	# (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
 #            [position.orientation.x,position.orientation.y,
 #             position.orientation.z, position.orientation.w])
	#print(position.position.x, position.position.y, position.position.z)


def controller():
	print("SEVEN")
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	tf_buffer = tf2_ros.Buffer()
	tf_listener = tf2_ros.TransformListener(tf_buffer)

	occuGridListener = rospy.Subscriber("/vis/map", Marker, callback)
	#tfListener = rospy.Subscriber("/tf", )
	#positionListener = rospy.Subscriber('/odom',Odometry,Position)
	#print(tfListener)
	r = rospy.Rate(10)
	start_yaw = 0

	rotating = False

	K = 0.4
	
	
	print("START")



	while not rospy.is_shutdown():
		try:
			control_command = Twist()
			pose = tf_buffer.lookup_transform("odom", "base_link", rospy.Time())
			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])

			if rotating:
				error = abs(abs((start_yaw - yaw)) - math.pi)
				if error > 0.1:
					if (rospy.is_shutdown()):
						break
					print(yaw)
					control_command.angular.z = K * error
				else:
					rotating = False

			else: 

				control_command.linear.x = .08
				last_x, last_y = 0,0 
				print("moving")	
				#print(position.position.x, position.position.y, position.position.z)
				sensor_x = pose.transform.translation.x
				sensor_y = pose.transform.translation.y
				xcoor = int(sensor_x * (51/20))
				ycoor = int(sensor_y * (51/20))

				


				###ROTATION###
				print("ANGLE")
				print(yaw)
				if ((xcoor,ycoor) in my_dict and my_dict[xcoor,ycoor] >= .6 ) or ( (xcoor + 1,ycoor) in my_dict  and my_dict[xcoor + 1, ycoor]>=.6):
					print("DETECTED")
					start_yaw = yaw
					#end_goal_yaw = yaw + 3.14
					rotating = True

					
					# while((yaw >= 0 and yaw <3.10) or (yaw < 0 and yaw > -3.10)):
					# 	if (rospy.is_shutdown()):
					# 		break
					# 	print(yaw)
					# 	control_command.linear.x = .00
					# 	control_command.angular.z = .12
					# 	pub.publish(control_command)
					# 	r.sleep()
					# break

					
			pub.publish(control_command)
				
					
			
			r.sleep()
		

	#while occuppancy grid infront is not
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			pass

	print("stop command")





if __name__ == '__main__':
	print("main")
	rospy.init_node('turtlebot_controller', anonymous=True)
	controller()