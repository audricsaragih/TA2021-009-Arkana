#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

#
# Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from test_tutorials.msg import Pwm
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid

def callbackPose(data):
    msgfix.header = data.header
    msgfix.pose = data.pose.pose

def callbackStop(data):
    goalpub.publish(msgfix)

def callbackMap(data):
    mappub = rospy.Publisher('map_continues', OccupancyGrid, queue_size=10)
    rateMap = rospy.Rate(6)
    while not rospy.is_shutdown():
        mappub.publish(data)
        rospy.loginfo('publish map')
        rateMap.sleep()

def listener():
    rospy.init_node('interface_node', anonymous=True)
    msgfix.header.stamp = rospy.Time.now()
    msgfix.header.frame_id = 'map'
    msgfix.pose.position.x = 0.0
    msgfix.pose.position.y = 0.0
    msgfix.pose.position.z = 0.0
    msgfix.pose.orientation.x = 0.0
    msgfix.pose.orientation.y = 0.0
    msgfix.pose.orientation.z = 0.0
    msgfix.pose.orientation.w = 0.0

    rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callbackPose)
    rospy.Subscriber("stop_goal_new", Twist, callbackStop)
    rospy.Subscriber("map", OccupancyGrid, callbackMap)

    #rospy.spin()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        posepub.publish(msgfix)
        #rospy.loginfo(msgfix)
        rate.sleep()

if __name__ == '__main__':
    try:
        #talker()
        posepub = rospy.Publisher('pose', PoseStamped, queue_size=10)
        goalpub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)

        msgfix = PoseStamped()
        #goal = PoseStamped()

        

        listener()
    except rospy.ROSInterruptException:
        pass
