#!/usr/bin/python
################################################################################
#
# Node to wrap the ParkingController class.
#
################################################################################

from ControllerV3 import ParkingController

import rospy
import sys

# if __name__ == "__main__":
#     rospy.init_node("mapping_node")

#     og = OccupancyGrid2d()
#     if not og.Initialize():
#         rospy.logerr("Failed to initialize the mapping node.")
#         sys.exit(1)

#     rospy.spin()

if __name__ == '__main__':
    print("main")
    rospy.init_node('parking_node', anonymous=True)

    pc = ParkingController()
    if not pc.Initialize():
         rospy.logerr("Failed to initialize the mapping node.")
         sys.exit(1)

    pc.controller()
    
