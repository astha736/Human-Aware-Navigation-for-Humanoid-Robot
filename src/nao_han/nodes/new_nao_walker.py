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
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class NaoWalker(NaoqiNode):
	def __init__(self):
		NaoqiNode.__init__(self, 'nao_walker')

		self.connectNaoQi()

		# walking pattern params:
		self.stepFrequency = rospy.get_param('~step_frequency', 0.5)

		# last: ROS subscriptions (after all vars are initialized)
		# /mobile_base/commands/velocity
		# /mobile_base/commands/velocity
		rospy.Subscriber("/mobile_base/commands/velocity", Twist, self.handleCmdVel, queue_size=1)
		rospy.Subscriber("cmd_pose", Pose2D, self.handleTargetPose, queue_size=1)
		self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
		self.vel_pub = rospy.Publisher("/nao/velocity", Twist, queue_size=10)
		self.seq = 0

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
		self.postureProxy.goToPosture("StandInit", 0.2)

	def getVelocityTwist(self):

		new_vel = Twist()

		try:
			result_twist = self.motionProxy.getRobotVelocity()

			
			new_vel.linear.x = result_twist[0]
			new_vel.linear.y = result_twist[1]
			new_vel.angular.z = result_twist[2]

			self.vel_pub.publish(new_vel)

			rospy.loginfo(result_twist)	

		except RuntimeError,e:
			# We have to assume there's no NaoQI running anymore => exit!
			rospy.logerr("Exception caught in getVelocityTwist:\n%s", e)
			rospy.signal_shutdown("No NaoQI available anymore")	

		return new_vel

	def getPositionOdom(self):

		self.seq += 1
		# result = []
		odom = Odometry()

		try:
			useSensorValues = False

			result_pose = self.motionProxy.getRobotPosition(useSensorValues)
			result_twist = self.motionProxy.getRobotVelocity()

			odom.header.seq = self.seq
			odom.header.stamp = rospy.get_rostime()
			rospy.loginfo(odom.header.stamp)
			odom.header.frame_id = "odom"
			odom.child_frame_id  = "base_link"
			odom.pose.pose.position.x = result_pose[0]
			odom.pose.pose.position.y = result_pose[1]

			roll = 0
			pitch = 0
			yaw = result_pose[2]
			quaternion = quaternion_from_euler(roll, pitch, yaw)

			odom.pose.pose.orientation.x = quaternion[0]
			odom.pose.pose.orientation.y = quaternion[1]
			odom.pose.pose.orientation.z = quaternion[2]
			odom.pose.pose.orientation.w = quaternion[3]


			odom.twist.twist.linear.x = result_twist[0]
			odom.twist.twist.linear.y = result_twist[1]
			odom.twist.twist.angular.z = result_twist[2]

			self.odom_pub.publish(odom)

		except RuntimeError,e:
			rospy.logerr("Exception caught in handleCmdVel:\n%s", e)
			rospy.signal_shutdown("No NaoQI available anymore")

		return odom

	def handleCmdVel(self, data):
		rospy.loginfo("Walk cmd_vel: %f %f %f, frequency %f", data.linear.x, data.linear.y, data.angular.z, self.stepFrequency)
		
		try:
			# self.motionProxy.moveToward(data.linear.x, data.linear.y, data.angular.z, [["Frequency", self.stepFrequency]])
			x = data.linear.x
			y = data.linear.y
			z = data.angular.z
			if abs(x) > 1:
				sign = x/abs(x)
				x = sign*1
			if abs(y) > 1:
				sign = y/abs(y)
				y = sign*1
			if abs(z) > 1:
				sign = z/abs(z)
				z = sign*1
			rospy.loginfo("Walk final: %f %f %f", x,y,z)

			self.motionProxy.move(x, y, z)

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
	try:

		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			#result_odom = walker.getPositionOdom()
			# rospy.loginfo(result)
			result_vel = walker.getVelocityTwist()
			rate.sleep()

		walker.motionProxy.moveToward(0,0, 0)
		walker.motionProxy.stopMove()
		walker.motionProxy.rest()
		rospy.loginfo("nao_walker stopped.")
	except RuntimeError,e:
		print "An error has been caught"
		print e
		# return False
	exit(0)
