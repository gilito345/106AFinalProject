#########

import math
import numpy as np
import rospy
import tf2_ros
import sys
import tf
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import ColorRGBA

MARKER_ID_DETECTION = 9

class ParkingController(object):
	def __init__(self):
		self._tf_buffer = tf2_ros.Buffer()
		self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
		self._my_dict = {}
		self._my_image = {}
		self._position = 0
		self._last_x = 0
		self._last_y = 0

		self._robot_x = 0
		self._robot_y = 0
		self._marker_x = 0
		self._marker_y = 0

		self.valid_spot = False

		self._front_occupied = False
		self._right_free = False

		self._right_cd = 0

		self.K = 0.4

	def Initialize(self):
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
		self.sub_odom_robot = rospy.Subscriber('/odom', Odometry, self.cbGetRobotOdom, queue_size = 1)
		self.sub_info_marker = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.cbGetMarkerOdom, queue_size = 1)
		self.imageListener = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.imagecallback)
		self.laserListener = rospy.Subscriber("/scan", LaserScan, self.processScan)
		return True

	def callback(self, data):
		for blocks in range(10201):
			self._my_dict[int(data.points[blocks].x * (101/20)) ,int( data.points[blocks].y * (101/20))] = data.colors[blocks].r

	def cbGetRobotOdom(self, robot_odom_msg):
		pos_x = robot_odom_msg.pose.pose.position.x
		pos_y = robot_odom_msg.pose.pose.position.y

		self._robot_x = pos_x
		self._robot_y = pos_y

	def cbGetMarkerOdom(self, markers_odom_msg):
		for marker_odom_msg in markers_odom_msg.markers:
			if marker_odom_msg.id == MARKER_ID_DETECTION:
				if self.valid_spot == False:
					self.valid_spot = True

				pos_x = marker_odom_msg.pose.pose.position.x
				pos_y = marker_odom_msg.pose.pose.position.y

				self._marker_x = pos_x
				self._marker_y = pos_y
				break

	def processScan(self, scan):
		front_avg = 0.
		right_avg = 0.
		for idx, r in enumerate(scan.ranges):
			if (idx < 25) or (idx >= 325):
				front_avg += r
			elif (240 <= idx < 300):
				right_avg += r

		if front_avg / 50. < 0.6:
			self._front_occupied = True
		else: 
			self._front_occupied = False

		if right_avg / 60. > 0.6 and self._right_cd < 0:
			self._right_free = True
		else: 
			self._right_free = False
			self._right_cd -= 1


	def imagecallback(self, data):
		#self._my_image = threshold_segment_naiva(data.data, 70, 200)
		self._my_image = data.data
		#print(sum(self._my_image)/len(self._my_image))

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

	def calcDist(self, x1, y1, x2, y2):
		return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)


	def controller(self):
		print("I AM A CAR")
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		r = rospy.Rate(10)
		start_yaw = 0

		startcount = 0

		startup = True
		rotating = False
		rightrotating = False
		leftrotating = False
		signcheck = False

		parking = False
		foundspot = False
		parkingcount = 0
		
		checkspot = False
		checkingcount = 0
		checkingsum = 0

		taggedspot = False
		
		print("START")



		while not rospy.is_shutdown():
			try:
				control_command = Twist()

				pose = self._tf_buffer.lookup_transform("odom", "base_link", rospy.Time())
				(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])

				if startup:
					control_command.linear.x = .03
					startcount += 1
					if (startcount == 50):
						startup = False
						print("startup done")

				elif signcheck:
					print("checking sign")
					if self.valid_spot:
						parking = True
						taggedspot = True

					else: 
						print("no sign")
						leftrotating = True
						start_yaw = yaw
						foundspot = False

					signcheck = False

				elif parking == True:
					print("robotx: " + str(self._robot_x) + " roboty: " + str(self._robot_y) + " markerx: " + str(self._marker_x) + " markery: " + str(self._marker_y))
					marker_pose = self._tf_buffer.lookup_transform("odom", "ar_marker_9", rospy.Time())
					error = self.calcDist(pose.transform.translation.x, pose.transform.translation.y, marker_pose.transform.translation.x, marker_pose.transform.translation.y)
					print(error)
					if error > 1:
						print("parking")
						control_command.linear.x = .03

					else:
						print("got into spot")
						parking = False
						leftrotating = True
						start_yaw = yaw

				elif rotating:
					# if start_yaw > 0:
					# 	start_yaw -= 2*math.pi
					# 	yaw -= 2*math.pi
					error = abs(abs((start_yaw - yaw)) - math.pi)
					if error > 0.05:
						if (rospy.is_shutdown()):
							break
						print("180yaw " + str(yaw) + " " + str(error))

						control_command.angular.z = self.K * error

					else:
						rotating = False

				elif rightrotating:
					if start_yaw < -0.5* math.pi:
						start_yaw += 2*math.pi
						yaw += 2*math.pi
					error = abs(abs((start_yaw - yaw))- 0.5*math.pi)
					if error > 0.05:
						if (rospy.is_shutdown()):
							break
						print("rightyaw " + str(yaw)+ " " + str(error))

						control_command.angular.z = self.K * -error

					else:
						rightrotating = False
						signcheck = True

				elif leftrotating:
					if start_yaw > 0.5* math.pi:
						start_yaw -= 2*math.pi
						yaw -= 2*math.pi
					error = abs(abs((start_yaw - yaw)) - 0.5*math.pi)
					if error > 0.05:
						if (rospy.is_shutdown()):
							break
						print("leftyaw " + str(yaw)+ " " + str(error))

						control_command.angular.z = self.K * error

					else:
						leftrotating = False
						self._right_cd = 20
						if (taggedspot == True):
							print("PARKING DONE")
							break

				else: 

					# if (taggedspot == False) & (foundspot == True):
					# 	print("invalid spot")
					# 	print("continuing search")
					# 	foundspot = False
					# 	start_yaw = yaw
					# 	leftrotating = True

					if (checkspot == True):
						control_command.linear.x = .026
					elif (foundspot == False):
						control_command.linear.x = .07
					elif (parking == True) or (taggedspot == False): 
						control_command.linear.x = 0.0
					elif (taggedspot == True):
						control_command.linear.x = .03
						parkingcount += 1
					
					print("moving")	
					#print(position.position.x, position.position.y, position.position.z)
					sensor_x = pose.transform.translation.x
					sensor_y = pose.transform.translation.y
					xcoor = int(sensor_x * (101/20))
					ycoor = int(sensor_y * (101/20))

					# #print(checkingcount)
					# if (checkingcount == 78):
					# 	print("checking spot")
					# 	print(checkingsum)

					# #detect gap on right
					# if  ((self.checkRightNotOccupied(xcoor, ycoor)) and (foundspot == False)) or  (checkingcount == 80):
					# 	checkspot = True
					# 	if (checkingcount < 80):
					# 		checkingsum += float(self.checkRightNotOccupied(xcoor, ycoor))
					# 		checkingcount += 1
					# 	else:
					# 		if (checkingsum > 20):
					# 			print("found spot!")
					# 			checkingsum = 0
					# 			checkingcount = 0
					# 			start_yaw = yaw
					# 			checkspot = False
					# 			rightrotating = True
					# 			foundspot = True
					# 		else: 
					# 			print("no spot!")
					# 			checkingsum = 0
					# 			checkingcount = 0
					# 			checkspot = False
					# elif (foundspot == False):
					# 	if (checkingcount < 80):
					# 		checkingsum += float(self.checkRightNotOccupied(xcoor, ycoor))
					# 		checkingcount += 1
					# 	else:
					# 		checkingsum = 0
					# 		checkingcount = 0 
					# 		print("no spot!")

					if self._right_free:
						print("found spot!")
						start_yaw = yaw
						checkspot = False
						rightrotating = True
						foundspot = True

					elif self._front_occupied:
						print("DETECTED")
						start_yaw = yaw
						rotating = True
						self._front_occupied = False
					else:
						self._right_cd = 20


					
					# #obstacle in front, no spot found => rotate 180
					# if (self.checkOccupied(xcoor, ycoor) or self.checkFrontOccupied(xcoor, ycoor)) and (foundspot == False):
					# 	print("DETECTED")
					# 	start_yaw = yaw

					# 	rotating = True

					# elif ((self.checkOccupied(xcoor, ycoor) or self.checkFrontOccupied(xcoor, ycoor)) or parkingcount==140) and (foundspot == True) and (taggedspot == True):
					# 	print("PARKING")
					# 	control_command.linear.x = .0
					# 	parkingcount = 0
					# 	start_yaw = yaw
					# 	parking = True
					# 	leftrotating = True

					self._last_x = sensor_x
					self._last_y = sensor_y
	
				pub.publish(control_command)	
				
				r.sleep()
			

		#while occuppancy grid infront is not
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				pass

		print("stop command initiated")





# if __name__ == '__main__':
# 	print("main")
# 	rospy.init_node('turtlebot_controller', anonymous=True)
# 	controller()