#!/usr/bin/env python
import rospy
##import RPi.GPIO as GPIO
import time

##import pwm message
from test_tutorials.msg import Pwm
from std_msgs.msg import String


class MotorDriver(object):

    def __init__(self, wheel_distance=0.098, wheel_diameter=0.066, i_BASE_PWM=50, i_MULTIPLIER_STANDARD=0.067, i_MULTIPLIER_PIVOT=0.65, simple_mode = True):
        """
        M1 = Right Wheel
        M2 = Left Wheel
        :param wheel_distance: Distance Between wheels in meters
        :param wheel_diameter: Diameter of the wheels in meters
        """
        #set message and publisher
        self.msg = Pwm()
        self.pub = rospy.Publisher('test_pwm', Pwm, queue_size=10)
	self.testpub = rospy.Publisher('chatter', String, queue_size=10)
	self.testmsg = 'Semangat'

        self.directions = ''
        

        self.PWM1 = 0
        self.PWM2 = 0
        self.BASE_PWM = i_BASE_PWM
        self.MAX_PWM = 255 ##edit skala max dari 100

        self.simple_mode = simple_mode

        # set dimensi roda dan chassis
        self._wheel_distance = wheel_distance
        self._wheel_radius = wheel_diameter / 2.0
        self.MULTIPLIER_STANDARD = i_MULTIPLIER_STANDARD
        self.MULTIPLIER_PIVOT = i_MULTIPLIER_PIVOT

    #fungsi publish nilai pwm
    def set_M1M2_speed(self, rpm_speedM1, rpm_speedM2, multiplier, directions): #tambah parameter directions
        self.PWM1 = min(int(((rpm_speedM1 * multiplier) * self.BASE_PWM)*255/100), self.MAX_PWM) #edit skala ke max 255
        self.PWM2 = min(int((rpm_speedM2 * multiplier * self.BASE_PWM)*255/100), self.MAX_PWM) ##edit skala ke max 255
        self.msg.directions = directions
        self.msg.pwm1 = self.PWM1
        self.msg.pwm2 = self.PWM2
        print("M1="+str(self.PWM1))
        print("M2="+str(self.PWM2))
        print("Direction="+directions)
        rospy.loginfo(self.msg)
	rospy.loginfo(self.testmsg)
	self.testpub.publish(self.testmsg)
        self.pub.publish(self.msg)
        
    #fungsi set kecepatan motor 1
    def set_M1_speed(self, rpm_speed, multiplier, directions):
        self.PWM1 = min(int(((rpm_speed * multiplier) * self.BASE_PWM)*255/100), self.MAX_PWM) ##edit skala ke max 255
        self.p1.ChangeDutyCycle(self.PWM1)
        print("M1="+str(self.PWM1))

    #fungsi set kecepatan motor 1
    def set_M2_speed(self, rpm_speed, multiplier, directions):
        self.PWM2 = min(int((rpm_speed * multiplier * self.BASE_PWM)*255/100), self.MAX_PWM) ##edit skala ke max 255
        self.p2.ChangeDutyCycle(self.PWM2)
        print("M2="+str(self.PWM2))

    #fungi hitung radius perputaran body
    def calculate_body_turn_radius(self, linear_speed, angular_speed):
        if angular_speed != 0.0:
            body_turn_radius = linear_speed / angular_speed
        else:
            # Not turning, infinite turn radius
            body_turn_radius = None
        return body_turn_radius

    #fungsi hitung radius perputaran roda
    def calculate_wheel_turn_radius(self, body_turn_radius, angular_speed, wheel):

        if body_turn_radius is not None:
            if wheel == "right":
                wheel_sign = 1
            elif wheel == "left":
                wheel_sign = -1
            else:
                assert False, "Wheel Name not supported, left or right only."

            wheel_turn_radius = body_turn_radius + ( wheel_sign * (self._wheel_distance / 2.0))
        else:
            wheel_turn_radius = None

        return wheel_turn_radius

    #fungsi hitung rpm roda
    def calculate_wheel_rpm(self, linear_speed, angular_speed, wheel_turn_radius):
        """
        Omega_wheel = Linear_Speed_Wheel / Wheel_Radius
        Linear_Speed_Wheel = Omega_Turn_Body * Radius_Turn_Wheel
        --> If there is NO Omega_Turn_Body, Linear_Speed_Wheel = Linear_Speed_Body
        :param angular_speed:
        :param wheel_turn_radius:
        :return:
        """
        if wheel_turn_radius is not None:
            # The robot is turning
            wheel_rpm = (angular_speed * wheel_turn_radius) / self._wheel_radius
        else:
            # Its not turning therefore the wheel speed is the same as the body
            wheel_rpm = linear_speed / self._wheel_radius

        return wheel_rpm

    #fungsi menentukan arah gerak roda
    def set_wheel_movement(self, right_wheel_rpm, left_wheel_rpm):
        if right_wheel_rpm > 0.0 and left_wheel_rpm > 0.0:
            if self.simple_mode:
                # We make it turn only on one wheel
                if right_wheel_rpm > left_wheel_rpm:
                    self.directions = 'right'
                if right_wheel_rpm < left_wheel_rpm:
                    self.directions = 'left'
                if right_wheel_rpm == left_wheel_rpm:
                    self.directions = 'forward'
            else:
                self.directions = 'forward'

            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD, self.directions) ##edit


        elif right_wheel_rpm > 0.0 and left_wheel_rpm == 0.0:
            self.directions = 'left'
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD, self.directions) ##edit

        elif right_wheel_rpm > 0.0 and left_wheel_rpm < 0.0:
            self.directions = 'pivot_left'
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_PIVOT, self.directions) ##edit

        elif right_wheel_rpm == 0.0 and left_wheel_rpm > 0.0:
            self.directions = 'right'
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD, self.directions) ##edit

        elif right_wheel_rpm < 0.0 and left_wheel_rpm > 0.0:
            self.directions = 'pivot_right'
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_PIVOT, self.directions) ##edit

        elif right_wheel_rpm < 0.0 and left_wheel_rpm < 0.0:
            if self.simple_mode:
                if abs(right_wheel_rpm) > abs(left_wheel_rpm):
                    self.directions = 'right_reverse'
                if abs(right_wheel_rpm) < abs(left_wheel_rpm):
                    self.directions = 'left_reverse'
                if right_wheel_rpm == left_wheel_rpm:
                    self.directions = 'reverse'
            else:
                self.directions = 'reverse'

            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD, self.directions) ##edit


        elif right_wheel_rpm == 0.0 and left_wheel_rpm == 0.0:
            self.directions = 'stop'
            self.set_M1M2_speed(abs(right_wheel_rpm), abs(left_wheel_rpm), self.MULTIPLIER_STANDARD, self.directions) ##edit

        else:
            assert False, "A case wasn't considered==>"+str(right_wheel_rpm)+","+str(left_wheel_rpm)
            pass

    #fungsi pengubahan /cmd_vel ke pergerakan roda
    def set_cmd_vel(self, linear_speed, angular_speed):

        body_turn_radius = self.calculate_body_turn_radius(linear_speed, angular_speed)

        wheel = "right"
        right_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                   angular_speed,
                                                                   wheel)

        wheel = "left"
        left_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                  angular_speed,
                                                                  wheel)

        right_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, right_wheel_turn_radius)
        left_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, left_wheel_turn_radius)


        self.set_wheel_movement(right_wheel_rpm, left_wheel_rpm)

