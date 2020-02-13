#!/usr/bin/env python

#
# ROS node to control Nao's walking engine (omniwalk and footsteps)
# This code is currently compatible to NaoQI version 1.6 or newer (latest
# tested: 1.12)
#
# Copyright 2009-2011 Armin Hornung & Stefan Osswald, University of Freiburg
# http://www.ros.org/wiki/nao
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    # Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#    # Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#    # Neither the name of the University of Freiburg nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy

# from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

# from std_srvs.srv import Empty, EmptyResponse
# from naoqi_bridge_msgs.srv import CmdPoseService, CmdVelService, CmdPoseServiceResponse, CmdVelServiceResponse, SetArmsEnabled, SetArmsEnabledResponse
# from humanoid_nav_msgs.msg import StepTarget
# from humanoid_nav_msgs.srv import StepTargetService, StepTargetServiceResponse

# # from nao_apps import startWalkPose
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

class ReMapper:
	def __init__(self):

		rospy.Subscriber("/mobile_base/velocity", Twist, self.handleCmdVel, queue_size=1)
		self.remap_velocity = rospy.Publisher("navigation_velocity_smoother/raw_cmd_vel", Twist, queue_size=10)
		self.seq = 0

		rospy.loginfo("nao_walker initialized")

	def handleCmdVel(self, data):
		# print("Walk cmd_vel: %f %f %f", data.linear.x, data.linear.y, data.angular.z)
		
		try:
			# self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z, [["Frequency", self.stepFrequency]])
			# self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z)
			newMap = data
			newMap.linear.x = newMap.linear.x
			newMap.linear.y = newMap.linear.y
			self.remap_velocity.publish(newMap)
		except RuntimeError,e:
			# We have to assume there's no NaoQI running anymore => exit!
			rospy.logerr("Exception caught in handleCmdVel:\n%s", e)
			# rospy.signal_shutdown("No NaoQI available anymore")


if __name__ == '__main__':
	rospy.init_node('remapper')
	walker = ReMapper()
	rospy.loginfo("remapper running...")
	try:
		rospy.spin()
	except RuntimeError,e:
		print "An error has been caught"
		print e
		# return False
	exit(0)
