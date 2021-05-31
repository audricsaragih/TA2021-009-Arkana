#!/usr/bin/env python

import sys
sys.path.insert(1,'/home/ubuntu/mybot_ws/src/beginner_tutorials/scripts/i2clibraries/')
from i2c_adxl345 import *
from i2c_itg3205 import *
from i2c_hmc5883l import *
from time import *
import math
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Vector3, Quaternion


class BerryIMUPublisher:
    """Reads IMU data from BerryIMU using I2C bus and publishes data to ROS
    
    """
    def __init__(self):
        rospy.init_node('berryimu', anonymous=True)
        self.RAD_TO_DEG = 180 / math.pi
        self.M_PI = 3.14159265358979323846
	self.ACC_TO_MS2 = -9.8  # 2^15 = 16G, 1G ~= 9.8m/s^2
        self.G_GAIN = 1
        self.GYRO_TO_RADS = 1 / self.RAD_TO_DEG
        self.pub = rospy.Publisher("imu/data_raw", Imu, queue_size=1000)
        self.rate = rospy.Rate(int(rospy.get_param("~poll_rate", 10)))

        # Special setup for virtual I2C device so doesn't conflict with actual device
        is_virtual = int(rospy.get_param("~is_virtual", 0))
        gyr_addr = int(rospy.get_param("~gyr_addr", 0x68))
        mag_addr = int(rospy.get_param("~mag_addr", 0x1e))
        acc_addr = int(rospy.get_param("~acc_addr", 0x53))


    def begin(self):
        """Keeps reading IMU data and publishing it to the topic imu_data
        
        """
        gyro_x_angle = 0.0
        gyro_y_angle = 0.0
        gyro_z_angle = 0.0
        last_yaw = 0
        first_run = True
        last_timestamp = rospy.get_time()

	adxl345 = i2c_adxl345(1)
        itg3205 = i2c_itg3205(1)
        hmc5883l = i2c_hmc5883l(1)

        while not rospy.is_shutdown():
            try:
                (itgready, dataready) = itg3205.getInterruptStatus ()
                hmc5883l.setContinuousMode ()
                hmc5883l.setDeclination (2,4)
                if dataready:
                  # Read the accelerometer,gyroscope and magnetometer values        
                  (gyr_x, gyr_y, gyr_z) = itg3205.getDegPerSecAxes ()
                  gyr_x += 1.46 
                  gyr_y -= 0.486956 #4.8
                  gyr_z -= 0.486956 #4.8

	          print ("gyr_x:" + str (gyr_x )) 
        	  print ("gyr_y:" + str (gyr_y )) 
	          print ("gyr_z:" + str (gyr_z )) 
	          print ("")

                  (acc_x, acc_y, acc_z) = adxl345.getAxes ()
		  acc_x += 0.03125
		  acc_y += 0.03125
	          print ("acc_x:" + str (acc_x ))
        	  print ("acc_y:" + str (acc_y ))
	          print ("acc_z:" + str (acc_z ))
	          print ("")

                  (mag_x, mag_y, mag_z) = hmc5883l.getAxes ()
                  print("Mag_X " + str(mag_x ))
                  print("Mag_Y " + str(mag_y ))
                  print("Mag_Z " + str(mag_z ))

                # Calculate loop Period(LP). How long between Gyro Reads
                period = (rospy.get_time() - last_timestamp)
                last_timestamp = rospy.get_time()

                # Convert Gyro raw to degrees per second
                rate_gyr_x = gyr_x * self.G_GAIN
                rate_gyr_y = gyr_y * self.G_GAIN
                rate_gyr_z = gyr_z * self.G_GAIN

                # Calculate the angles from the gyro.
                gyro_x_angle += rate_gyr_x * period
                gyro_y_angle += rate_gyr_y * period
                gyro_z_angle += rate_gyr_z * period
		

                # Calculate heading
                #heading = math.atan2(mag_y, mag_x) * self.RAD_TO_DEG
                (heading, minutes) = hmc5883l.getHeading()

                # Only have our heading between 0 and 360
                #if heading < 0:
                #    heading += 360

                # Normalize accelerometer raw values.
                norm_acc = math.sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z)
                if norm_acc > 1e-9:
                    acc_x_norm = acc_x / norm_acc
                    acc_y_norm = acc_y / norm_acc

                    # Calculate pitch and roll
                    # rospy.loginfo("acc_x_norm=%f norm=%f" % (acc_x_norm, norm))
                    pitch = math.asin(acc_x_norm)
                    roll = -math.asin(acc_y_norm / math.cos(pitch))

                    # Calculate the new tilt compensated values
                    mag_x_comp = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)
                    mag_y_comp = mag_x * math.sin(roll) * math.sin(pitch) + mag_y * math.cos(roll) - mag_z * math.sin(roll) * math.cos(pitch)

                    # Calculate tilt compensated heading
                    #tilt_compensated_heading = math.atan2(mag_y_comp, mag_x_comp) * self.RAD_TO_DEG
                    tilt_compensated_heading = heading

                    if tilt_compensated_heading < 0:
                        tilt_compensated_heading += 360

                    msg = Imu()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = "/imu"

                    # Convert from euler to quaternion for ROS
                    #if first_run:
                    #    q = quaternion_from_euler(roll * self.RAD_TO_DEG, pitch * self.RAD_TO_DEG, 0)
                    #    first_run = False
                    #else:
                    #    q = quaternion_from_euler(roll * self.RAD_TO_DEG, pitch * self.RAD_TO_DEG, tilt_compensated_heading)# - last_yaw)'''

                    last_yaw = tilt_compensated_heading
                    print("Heading " + str(last_yaw ))

                    #msg.orientation = Quaternion(q[0], q[1], q[2], q[3])
                    # TODO: measure covariance
                    msg.angular_velocity = Vector3(rate_gyr_x * self.GYRO_TO_RADS, rate_gyr_y * self.GYRO_TO_RADS,
                                                   rate_gyr_z * self.GYRO_TO_RADS)

                    msg.linear_acceleration = Vector3((acc_x * self.ACC_TO_MS2), (acc_y * self.ACC_TO_MS2),
                                                      acc_z * self.ACC_TO_MS2)

                    rospy.loginfo_throttle(1, "Heading: %s" % tilt_compensated_heading)

                    self.pub.publish(msg)
            except IOError as e:
                rospy.logwarn(e)
            self.rate.sleep()



if __name__ == "__main__":
    try:
        berry = BerryIMUPublisher()
        berry.begin()
    except rospy.ROSInterruptException:
        pass




