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

class ParkingController(object):
	def __init__(self):
		self._tf_buffer = tf2_ros.Buffer()
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
		self._my_dict = {}
		self._position = 0
		self._last_x = 0
		self._last_y = 0

		self.K = 0.4

	def Initialize(self):
        #self._name = rospy.get_name() + "/grid_map_2d"

        # Load parameters.
        # if not self.LoadParameters():
        #     rospy.logerr("%s: Error loading parameters.", self._name)
        #     return False

        # Register callbacks.
		if not self.RegisterCallbacks():
			rospy.logerr("%s: Error registering callbacks.", self._name)
			return False

		self._initialized = True
		return True

	def RegisterCallbacks(self):
		self.occuGridListener = rospy.Subscriber("/vis/map", Marker, self.callback)
		return True


	def callback(self, data):
		for blocks in range(2601):
			self._my_dict[int(data.points[blocks].x * (51/20)) ,int( data.points[blocks].y * (51/20))] = data.colors[blocks].r

	def getVelDir(self):
		pose = self._tf_buffer.lookup_transform("odom", "base_link", rospy.Time())
		dx = pose.transform.translation.x - self._last_x
		dy = pose.transform.translation.y - self._last_y

		if (abs(dx) < abs(dy)): #moving more up/down than left/right
			if (dy > 0): #moving up
				return 0, 1
			else: #moving down
				return 0, -1
		else: 
			if (dx > 0): #moving right
				return 1, 0
			else: #moving left
				return -1, 0

	def checkOccupied(self, xCoor, yCoor):
		return (xCoor,yCoor) in self._my_dict and self._my_dict[xCoor,yCoor] >= .4

	def checkNotOccupied(self, xCoor, yCoor):
		return (xCoor,yCoor) in self._my_dict and self._my_dict[xCoor,yCoor] <= .2

	def checkFrontOccupied(self, xCoor, yCoor):
		dx, dy = self.getVelDir()

		if dx == 0:
			return self.checkOccupied(xCoor, yCoor + dy) 
			#or self.checkOccupied(xCoor - 1, yCoor + dy) or self.checkOccupied(xCoor + 1, yCoor + dy)
		elif dy == 0:
			return self.checkOccupied(xCoor + dx, yCoor) 
			#or self.checkOccupied(xCoor + dx, yCoor - 1) or self.checkOccupied(xCoor + dx, yCoor + 1)
		else: 
			return self.checkOccupied(xCoor + dx, yCoor + dy) 

	def checkHalfStepOccupied(self, xCoor, yCoor):
		dx, dy = self.getVelDir()

		if dx == 0:
			return self.checkOccupied(xCoor, round(yCoor + 0.5*dy, 0)) 
			#or self.checkOccupied(xCoor - 1, yCoor + dy) or self.checkOccupied(xCoor + 1, yCoor + dy)
		else:
			return self.checkOccupied(round(xCoor + 0.5*dx, 0), yCoor) 
			#or self.checkOccupied(xCoor + dx, yCoor - 1) or self.checkOccupied(xCoor + dx, yCoor + 1)

	def checkRightOccupied(self, xCoor, yCoor):
		dx, dy = self.getVelDir()

		if dx == 0: #going up/down
			return self.checkOccupied(xCoor + dy, yCoor) 
			#or self.checkOccupied(xCoor + dy, yCoor + 1) or self.checkOccupied(xCoor + dy, yCoor - 1)
		else: #going left/right
			return self.checkOccupied(xCoor, yCoor + dx) 
			#or self.checkOccupied(xCoor + 1, yCoor + dx) or self.checkOccupied(xCoor - 1, yCoor + dx)



	def checkRightNotOccupied(self, xCoor, yCoor):
		dx, dy = self.getVelDir()

		if dx == 0: #going up/down
			return self.checkNotOccupied(xCoor + dy, yCoor) 
			#or self.checkOccupied(xCoor + dy, yCoor + 1) or self.checkOccupied(xCoor + dy, yCoor - 1)
		else: #going left/right
			return self.checkNotOccupied(xCoor, yCoor + dx) 
			#or self.checkOccupied(xCoor + 1, yCoor + dx) or self.checkOccupied(xCoor - 1, yCoor + dx)


	def controller(self):
		print("SEVEN")
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		r = rospy.Rate(10)
		start_yaw = 0

		rotating = False
		rightrotating = False
		leftrotating = False
		parking = False
		foundspot = False
		
		print("START")



		while not rospy.is_shutdown():
			try:
				control_command = Twist()
				pose = self._tf_buffer.lookup_transform("odom", "base_link", rospy.Time())
				(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])

				if rotating:
					error = abs(abs((start_yaw - yaw)) - math.pi)
					if error > 0.05:
						if (rospy.is_shutdown()):
							break
						#print(yaw)

						control_command.angular.z = self.K * error

					else:
						rotating = False

				elif rightrotating:
					error = abs(abs((start_yaw - yaw)) - 0.5*math.pi)
					if error > 0.05:
						if (rospy.is_shutdown()):
							break
						#print(yaw)

						control_command.angular.z = self.K * -error

					else:
						rightrotating = False

				elif leftrotating:
					error = abs(abs((start_yaw - yaw)) - 0.5*math.pi)
					if error > 0.05:
						if (rospy.is_shutdown()):
							break
						#print(yaw)

						control_command.angular.z = self.K * error

					else:
						leftrotating = False
						print("PARKING DONE")
						break


				else: 


					if (foundspot == False):
						control_command.linear.x = .07
					elif (parking == True): 
						control_command.linear.x = 0.0
					else:
						control_command.linear.x = .02
					
					print("moving")	
					#print(position.position.x, position.position.y, position.position.z)
					sensor_x = pose.transform.translation.x
					sensor_y = pose.transform.translation.y
					xcoor = int(sensor_x * (51/20))
					ycoor = int(sensor_y * (51/20))

					if  (self.checkRightNotOccupied(xcoor, ycoor)) and (foundspot == False):
						print("found spot!")
						start_yaw = yaw
						rightrotating = True
						foundspot = True
					elif (foundspot == False):
						print("no spot!")

					

					
					#print("ANGLE")
					#print(yaw)
					if (self.checkOccupied(xcoor, ycoor) or self.checkFrontOccupied(xcoor, ycoor)) and (foundspot == False):
						print("DETECTED")
						start_yaw = yaw

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
					elif (self.checkOccupied(xcoor, ycoor) or self.checkFrontOccupied(xcoor, ycoor)) and (foundspot == True):
						print("PARKING")
						control_command.linear.x = .0
						start_yaw = yaw
						parking = True
						leftrotating = True

					self._last_x = xcoor
					self._last_y = ycoor

						
				pub.publish(control_command)
					
						
				
				r.sleep()
			

		#while occuppancy grid infront is not
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				pass

		print("stop command")





# if __name__ == '__main__':
# 	print("main")
# 	rospy.init_node('turtlebot_controller', anonymous=True)
# 	controller()