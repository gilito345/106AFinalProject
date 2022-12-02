#########


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospy
import tf2_ros
import sys
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA
marker = Marker()
position = Pose()

def callback(data):
	global marker
	marker.points = data.points
	marker.pose = data.pose
	#print(data.points)

	#print(data.pose)

def callback1(data):
	print(data.linear.x)
	
def Position(odom):
	global position
	position = odom.pose.pose
	#print(position.position.x, position.position.y, position.position.z)


def controller():
	print("SEVEN")
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	occuGridListener = rospy.Subscriber("/vis/map", Marker, callback)
	#tfListener = rospy.Subscriber("/tf", )
	positionListener = rospy.Subscriber('/odom',Odometry,Position)
	#print(tfListener)
	r = rospy.Rate(2)
	control_command = Twist()
	control_command.linear.x = .08

	while not rospy.is_shutdown():
		try:
			pub.publish(control_command)
			print("moving")	
			print(position.position.x, position.position.y, position.position.z)
			xcoor = int(position.position.x * (51/20))
			ycoor = int(position.position.y * (51/20))
			print(xcoor,ycoor)
			r.sleep()
		

	#while occuppancy grid infront is not
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			pass

	print("stop command")





if __name__ == '__main__':
	print("main")
	rospy.init_node('turtlebot_controller', anonymous=True)
	controller()