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

from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

from std_srvs.srv import Empty, EmptyResponse
from naoqi_bridge_msgs.srv import CmdPoseService, CmdVelService, CmdPoseServiceResponse, CmdVelServiceResponse, SetArmsEnabled, SetArmsEnabledResponse
from humanoid_nav_msgs.msg import StepTarget
from humanoid_nav_msgs.srv import StepTargetService, StepTargetServiceResponse

# from nao_apps import startWalkPose

class NaoWalker(NaoqiNode):
	def __init__(self):
		NaoqiNode.__init__(self, 'nao_walker')

		self.connectNaoQi()

		# walking pattern params:
		self.stepFrequency = rospy.get_param('~step_frequency', 0.2)

		# last: ROS subscriptions (after all vars are initialized)
		rospy.Subscriber("cmd_vel", Twist, self.handleCmdVel, queue_size=1)
		rospy.Subscriber("cmd_pose", Pose2D, self.handleTargetPose, queue_size=1)

		# self.say("Walker online")

		rospy.loginfo("nao_walker initialized")

	def connectNaoQi(self):
		'''(re-) connect to NaoQI'''
		rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

		# get the proxy 
		self.motionProxy = self.get_proxy("ALMotion")
		self.postureProxy = self.get_proxy("ALRobotPosture")
		if self.motionProxy is None or self.postureProxy is None:
			rospy.logerr("Unable to connect to ALMotion or ALRobotPosture")
			exit(1)
		
		# wake up the robot
		self.motionProxy.wakeUp()
		self.postureProxy.goToPosture("StandInit", 0.5)

	def handleCmdVel(self, data):
		rospy.logdebug("Walk cmd_vel: %f %f %f, frequency %f", data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
		
		try:
			# self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z, [["Frequency", self.stepFrequency]])
			self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z)
		except RuntimeError,e:
			# We have to assume there's no NaoQI running anymore => exit!
			rospy.logerr("Exception caught in handleCmdVel:\n%s", e)
			rospy.signal_shutdown("No NaoQI available anymore")

	def handleTargetPose(self, data, post=True):
		"""handles cmd_pose requests, walks to (x,y,theta) in robot coordinate system"""

		# moveTo is a blocking call
		rospy.logdebug("Walk target_pose: %f %f %f", data.x,
				data.y, data.theta)

		try:
			self.motionProxy.moveTo(data.x, data.y, data.theta)
			return True
		except RuntimeError,e:
			rospy.logerr("Exception caught in handleTargetPose:\n%s", e)
			return False

if __name__ == '__main__':
	walker = NaoWalker()
	rospy.loginfo("nao_walker running...")
	rospy.spin()
	rospy.loginfo("nao_walker stopping...")
	try:
		walker.motionProxy.moveToward(0,0, 0)
		walker.motionProxy.stopMove()
		walker.motionProxy.rest()
		rospy.loginfo("nao_walker stopped.")
	except RuntimeError,e:
		print "An error has been caught"
		print e
		# return False
	exit(0)
