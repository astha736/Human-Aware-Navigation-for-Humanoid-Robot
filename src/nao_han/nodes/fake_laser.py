#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

import math
import numpy as np

from geometry_msgs.msg import Pose2D


# Header header            # timestamp in the header is the acquisition time of 
#                          # the first ray in the scan.
#                          #
#                          # in frame frame_id, angles are measured around 
#                          # the positive Z axis (counterclockwise, if Z is up)
#                          # with zero angle being forward along the x axis
						 
# float32 angle_min        # start angle of the scan [rad]
# float32 angle_max        # end angle of the scan [rad]
# float32 angle_increment  # angular distance between measurements [rad]

# float32 time_increment   # time between measurements [seconds] - if your scanner
#                          # is moving, this will be used in interpolating position
#                          # of 3d points
# float32 scan_time        # time between scans [seconds]

# float32 range_min        # minimum range value [m]
# float32 range_max        # maximum range value [m]

# float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
# float32[] intensities    # intensity data [device-specific units].  If your
#                          # device does not provide intensities, please leave
#                          # the array empty.

from nav_msgs.msg import Odometry

laserScan = LaserScan()  
laserScan.header.frame_id = "base_link" 
laserScan.angle_min = -3.14
laserScan.angle_max = 3.13
laserScan.range_min = 0
laserScan.range_max = 10
laserScan.angle_increment = 0.01


def laserScan_cal():
	global laserScan
	# ang = math.atan2(y,x)
	# ang = round(ang,2)
	size_output = int((laserScan.angle_max - laserScan.angle_min)*100 + 1)
	output_scan = np.empty(size_output,float) 

	output_scan[:] = 10.0

	return output_scan


def odom_callback(odom_data):
	seq = 1
	prevTime = 0
	 
	laserScan.header.seq = seq
	laserScan.header.stamp =  odom_data.header.stamp
	
	rospy.loginfo("Time " )
	rospy.loginfo(laserScan.header.stamp)

	laserScan.ranges = laserScan_cal()
	seq += 1

	fakescan_pub.publish(laserScan)



if __name__ == '__main__':
	try:

		rospy.init_node('nao_fake_laser', log_level=rospy.DEBUG)
		fakescan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
		# ball_pose_sub = rospy.Subscriber('/ball_pose_pub', Pose2D, ball_pose_callback,  queue_size=10)
		odom_sub = rospy.Subscriber("/odom",Odometry, odom_callback,  queue_size = 1)

		rate = rospy.Rate(5) #hz

		while not rospy.is_shutdown():
			# rospy.spin()
			rospy.loginfo(rospy.get_rostime())
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
