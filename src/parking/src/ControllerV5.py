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

		self._parking_dist = 0

		self._front_dist = 0
		self._back_dist = 0
		self._right_dist = 0

		self.valid_spot = False
		self._parking = False

		self._state = 0

		self.K = 0.45

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
				if self.valid_spot == False and self._state == 5:
					self.valid_spot = True

				pos_x = marker_odom_msg.pose.pose.position.x
				pos_y = marker_odom_msg.pose.pose.position.y

				self._marker_x = pos_x
				self._marker_y = pos_y
				break

	def processScan(self, scan):
		size = len(scan.ranges)
		right_idx = int(size * 3/4)
		back_idx = int(size / 2) 
		self._front_dist = scan.ranges[0]
		self._right_dist = scan.ranges[right_idx]
		self._back_dist = scan.ranges[back_idx]

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
		return (xCoor,yCoor) in self._my_dict and self._my_dict[xCoor,yCoor] >= .6

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

	def shutdown(self):
		print("Shutting down. cmd_vel will be 0") 


	def controller(self):
		print("I AM A CAR")
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		r = rospy.Rate(10)
		start_yaw = 0

		startcount = 0
		checkcounter = 0

		# startup = True
		# rotating = False
		# rightrotating = False
		# leftrotating = False
		# signcheck = False

		# parking = False
		# foundspot = False
		# parkingcount = 0
		
		checkspot = False
		checkingcount = 0
		checkingsum = 0
		deadend_count = 0

		# taggedspot = False
		
		print("START")



		while not rospy.is_shutdown():
			try:
				control_command = Twist()

				pose = self._tf_buffer.lookup_transform("odom", "base_link", rospy.Time())
				(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [pose.transform.rotation.x, pose.transform.rotation.y,
             pose.transform.rotation.z, pose.transform.rotation.w])

				if self._state == 0: #initializing 
					control_command.linear.x = .03
					startcount += 1
					if (startcount == 30):
						# startup = False
						print("startup done")
						self._state = 1

				elif self._state == 1: #forward patrolling
					control_command.linear.x = 0.07
					if checkspot or deadend_count > 10:
						control_command.linear.x = 0.03

					sensor_x = pose.transform.translation.x
					sensor_y = pose.transform.translation.y
					xcoor = int(sensor_x * (101/20))
					ycoor = int(sensor_y * (101/20))



					#gap on right, checking to verify
					if  (self.checkRightNotOccupied(xcoor, ycoor)) or  (checkingcount > 80):
						checkspot = True
						if (checkingcount < 80):
							checkingsum += float(self.checkRightNotOccupied(xcoor, ycoor))
							checkingcount += 1
						else:
							if (checkingsum > 35):
								print("found spot!")
								checkingsum = 0
								checkingcount = 0
								start_yaw = yaw
								yaw_offset = 0
								checkspot = False
								self._state = 3
								#foundspot = True
							else: 
								print("no spot!")
								checkingsum = 0
								checkingcount = 0
								checkspot = False

					#self.checkOccupied(xcoor, ycoor) or 
					#obstacle in front => rotate 180
					#if (self.checkFrontOccupied(xcoor, ycoor)):
					#	print("DETECTED")
					#	start_yaw = yaw
					#	yaw_offset = 0
					#	self._state = 2

					if (self._front_dist < 1):
						print("obstacle detected for " + str(deadend_count) + " iterations")
						deadend_count += 1
					else:
						deadend_count -= 1
					if(deadend_count >= 30):
						print("deadend found, turning around")
						start_yaw = yaw
						yaw_offset = 0
						deadend_count = 0
						self._state = 2

					self._last_x = sensor_x
					self._last_y = sensor_y

				elif self._state == 2: #180 degree turn 
					if start_yaw > 0:
						start_yaw -= 2*math.pi
						yaw_offset = -2*math.pi
						#yaw -= 2*math.pi
					error = abs(abs((start_yaw - (yaw + yaw_offset))) - math.pi)
					if error > 0.05:
						if (rospy.is_shutdown()):
							break
						#print("180yaw " + str(yaw) + " " + str(error))

						control_command.angular.z = self.K * error

					else:
						self._state = 1

				elif self._state == 3: #right turn
					if start_yaw < -0.5* math.pi:
						start_yaw += 2*math.pi
						yaw_offset = 2*math.pi
						#yaw += 2*math.pi
					error = abs(abs((start_yaw - (yaw + yaw_offset)))- 0.5*math.pi)
					if error > 0.05:
						if (rospy.is_shutdown()):
							break
						#print("rightyaw " + str(yaw)+ " " + str(error))

						control_command.angular.z = self.K * -error
					else:
						print("done right turn")
						self._state = 5
						checkcounter = 0

				elif self._state == 4: #left turn
					if start_yaw > 0.5* math.pi:
						start_yaw -= 2*math.pi
						yaw_offset = -2*math.pi
						#yaw -= 2*math.pi
					error = abs(abs((start_yaw - (yaw + yaw_offset))) - 0.5*math.pi)
					if error > 0.05:
						if (rospy.is_shutdown()):
							break
						#print("leftyaw " + str(yaw)+ " " + str(error))

						control_command.angular.z = self.K * error

					else:
						if self._parking:
							self._state = 7
							print("PARKING DONE")
						else:
							self._state = 1

				elif self._state == 5: #check for sign
					print("checking sign")
					if self.valid_spot:
						print("valid spot!")
						self._state = 6
						self.initial_robot_pose_x = self._robot_x
						self.initial_robot_pose_y = self._robot_y
						self._parking_dist = self.calcDist(self.initial_robot_pose_x, self.initial_robot_pose_y, self._marker_x, self._marker_y)
						self._parking = True
						#taggedspot = True
					elif checkcounter == 10:
						print("no sign")
						self._state = 4
						#leftrotating = True
						start_yaw = yaw
						yaw_offset = 0
						#foundspot = False

					else: 
						checkcounter += 1
						

				elif self._state == 6: #parking in
					#print("robotx: " + str(self._robot_x) + " roboty: " + str(self._robot_y) + " markerx: " + str(self._marker_x) + " markery: " + str(self._marker_y))
					#dist_from_start = self.calcDist(self._robot_x, self._robot_y, self.initial_robot_pose_x, self.initial_robot_pose_y)
					#error = self._parking_dist - dist_from_start
					#print(str(self._parking_dist) + " " + str(error))
					# if error > 0.5:
					# 	print("parking")
					# 	control_command.linear.x = .03

					# else:
					# 	print("got into spot")
					# 	self._state == 4
					# 	#leftrotating = True
					# 	start_yaw = yaw
					# 	yaw_offset = 0

					if (self._front_dist >= 0.22):
						print("approaching spot")
						control_command.linear.x = 0.03
					else:
						print("got into spot")
						self._state = 4
						#leftrotating = True
						start_yaw = yaw
						yaw_offset = 0


					self._last_x = pose.transform.translation.x
					self._last_y = pose.transform.translation.y

					# elif ((self.checkOccupied(xcoor, ycoor) or self.checkFrontOccupied(xcoor, ycoor)) or parkingcount==140) and (foundspot == True) and (taggedspot == True):
					# 	print("PARKING")
					# 	control_command.linear.x = .0
					# 	parkingcount = 0
					# 	start_yaw = yaw
					# 	parking = True
					# 	leftrotating = True

				elif self._state == 7: #back up
					if (self._back_dist >= 0.2):
						print("backing up")
						control_command.linear.x = -0.03
					else:
						print("DONE PARKING")
						rospy.on_shutdown(self.shutdown)
						break



				# elif signcheck:
				# 	print("checking sign")
				# 	if self.valid_spot:
				# 		parking = True
				# 		taggedspot = True

				# 	else: 
				# 		print("no sign")
				# 		leftrotating = True
				# 		start_yaw = yaw
				# 		foundspot = False

				# 	signcheck = False

				# elif parking == True:
				# 	print("robotx: " + str(self._robot_x) + " roboty: " + str(self._robot_y) + " markerx: " + str(self._marker_x) + " markery: " + str(self._marker_y))
				# 	marker_pose = self._tf_buffer.lookup_transform("odom", "ar_marker_9", rospy.Time())
				# 	error = self.calcDist(pose.transform.translation.x, pose.transform.translation.y, marker_pose.transform.translation.x, marker_pose.transform.translation.y)
				# 	print(error)
				# 	if error > 1:
				# 		print("parking")
				# 		control_command.linear.x = .03

				# 	else:
				# 		print("got into spot")
				# 		parking = False
				# 		leftrotating = True
				# 		start_yaw = yaw

				# elif rotating:
				# 	if start_yaw > 0:
				# 		start_yaw -= 2*math.pi
				# 		yaw -= 2*math.pi
				# 	error = abs(abs((start_yaw - yaw)) - math.pi)
				# 	if error > 0.05:
				# 		if (rospy.is_shutdown()):
				# 			break
				# 		print("180yaw " + str(yaw) + " " + str(error))

				# 		control_command.angular.z = self.K * error

				# 	else:
				# 		rotating = False

				# elif rightrotating:
				# 	if start_yaw < -0.5* math.pi:
				# 		start_yaw += 2*math.pi
				# 		yaw += 2*math.pi
				# 	error = abs(abs((start_yaw - yaw))- 0.5*math.pi)
				# 	if error > 0.05:
				# 		if (rospy.is_shutdown()):
				# 			break
				# 		print("rightyaw " + str(yaw)+ " " + str(error))

				# 		control_command.angular.z = self.K * -error

				# 	else:
				# 		rightrotating = False
				# 		signcheck = True

				# elif leftrotating:
				# 	if start_yaw > 0.5* math.pi:
				# 		start_yaw -= 2*math.pi
				# 		yaw -= 2*math.pi
				# 	error = abs(abs((start_yaw - yaw)) - 0.5*math.pi)
				# 	if error > 0.05:
				# 		if (rospy.is_shutdown()):
				# 			break
				# 		print("leftyaw " + str(yaw)+ " " + str(error))

				# 		control_command.angular.z = self.K * error

				# 	else:
				# 		leftrotating = False
				# 		if (taggedspot == True):
				# 			print("PARKING DONE")
				# 			break

				# else: 

				# 	# if (taggedspot == False) & (foundspot == True):
				# 	# 	print("invalid spot")
				# 	# 	print("continuing search")
				# 	# 	foundspot = False
				# 	# 	start_yaw = yaw
				# 	# 	leftrotating = True

				# 	if (checkspot == True):
				# 		control_command.linear.x = .026
				# 	elif (foundspot == False):
				# 		control_command.linear.x = .07
				# 	elif (parking == True) or (taggedspot == False): 
				# 		control_command.linear.x = 0.0
				# 	elif (taggedspot == True):
				# 		control_command.linear.x = .03
				# 		parkingcount += 1
					
				# 	print("moving")	
				# 	#print(position.position.x, position.position.y, position.position.z)
				# 	sensor_x = pose.transform.translation.x
				# 	sensor_y = pose.transform.translation.y
				# 	xcoor = int(sensor_x * (101/20))
				# 	ycoor = int(sensor_y * (101/20))

				# 	#print(checkingcount)
				# 	if (checkingcount == 78):
				# 		print("checking spot")
				# 		print(checkingsum)

				# 	#detect gap on right
				# 	if  ((self.checkRightNotOccupied(xcoor, ycoor)) and (foundspot == False)) or  (checkingcount == 80):
				# 		checkspot = True
				# 		if (checkingcount < 80):
				# 			checkingsum += float(self.checkRightNotOccupied(xcoor, ycoor))
				# 			checkingcount += 1
				# 		else:
				# 			if (checkingsum > 20):
				# 				print("found spot!")
				# 				checkingsum = 0
				# 				checkingcount = 0
				# 				start_yaw = yaw
				# 				checkspot = False
				# 				rightrotating = True
				# 				foundspot = True
				# 			else: 
				# 				print("no spot!")
				# 				checkingsum = 0
				# 				checkingcount = 0
				# 				checkspot = False
				# 	elif (foundspot == False):
				# 		if (checkingcount < 80):
				# 			checkingsum += float(self.checkRightNotOccupied(xcoor, ycoor))
				# 			checkingcount += 1
				# 		else:
				# 			checkingsum = 0
				# 			checkingcount = 0 
				# 			print("no spot!")

					
				# 	#obstacle in front, no spot found => rotate 180
				# 	if (self.checkOccupied(xcoor, ycoor) or self.checkFrontOccupied(xcoor, ycoor)) and (foundspot == False):
				# 		print("DETECTED")
				# 		start_yaw = yaw

				# 		rotating = True

				# 	elif ((self.checkOccupied(xcoor, ycoor) or self.checkFrontOccupied(xcoor, ycoor)) or parkingcount==140) and (foundspot == True) and (taggedspot == True):
				# 		print("PARKING")
				# 		control_command.linear.x = .0
				# 		parkingcount = 0
				# 		start_yaw = yaw
				# 		parking = True
				# 		leftrotating = True

				# 	self._last_x = xcoor
				# 	self._last_y = ycoor
	
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