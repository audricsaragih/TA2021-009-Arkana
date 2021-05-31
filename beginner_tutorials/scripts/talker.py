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

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from test_tutorials.msg import Pwm

def talker():
    pub = rospy.Publisher('battery', String, queue_size=10)
    #testpub = rospy.Publisher('testpwm', Pwm, queue_size=10)
    #msg = Pwm()
    #msg.pwm1 = 0
    #msg.pwm2 = 1

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        #rate.sleep()
        #msg.pwm1 += 1
        #msg.pwm2 += 1
        #rospy.loginfo(msg)
        #testpub.publish(msg)
        #rate.sleep()
    rate.sleep()

    pub.publish(" 24.3")
    rate.sleep()
    pub.publish(" 24.1")
    rate.sleep()
    pub.publish(" 24.4")
    rate.sleep()
    pub.publish(" 24.2")
    rate.sleep()
    pub.publish(" 24.4")
    rate.sleep()
    pub.publish(" 24.4")
    rate.sleep()
    pub.publish(" 24.5")
    rate.sleep()
    pub.publish(" 24.3")
    rate.sleep()
    pub.publish(" 24.1")
    rate.sleep()
    pub.publish(" 24.2")
    rate.sleep()
    pub.publish(" 24.4")
    rate.sleep()
    pub.publish(" 24.2")
    rate.sleep()
    pub.publish(" 24.3")
    rate.sleep()
    pub.publish(" 24.2")
    rate.sleep()
    pub.publish(" 24.4")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
