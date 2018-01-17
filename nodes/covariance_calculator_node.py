#!/usr/bin/env python
import numpy
import rospy
import time
import collections
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
from covariance_calculator import covariance_calculator

class CovarianceCalculatorNode():

    def __init__(self):
        
        self.odom_topics = []
        self.odom_calculators = []
        self.imu_topics = []
        self.imu_calculators = []
        self.gps_topics = []
        self.gps_calculators = []
        
        count = 0
        while rospy.has_param("odom"+str(count)):
            self.odom_topics.append(rospy.get_param("odom" + str(count)))    
            self.odom_calculators.append(CovarianceCalculator())
            rospy.Subscriber(self.odom_topics[count], Odometry, self.odom_calculators[count].insert_odom)
            count = count + 1 
       
        count = 0
        while rospy.has_param("imu"+str(count)):
            self.imu_topics.append(rospy.get_param("imu" + str(count)))
            self.imu_calculators.append(CovarianceCalculator())
            rospy.Subscriber(self.imu_topics[count], Imu, self.imu_calculators[count].insert_imu)
            count = count +1
         
        count = 0
        while rospy.has_param("imu"+str(count)):
            self.gps_topics.append(rospy.get_param("gps" + str(count)))
            self.gps_calculators.append(CovarianceCalculator())
            rospy.Subscriber(self.gps_topics[count], NavSatFix, self.gps_calculators[count].insert_gps)
            count = count + 1 

        self.output_file_name = rospy.get_param("output_file_name", "covariance_out.txt")

    def compute_cov(self):
        self.odom_msgs_w_covariance = []
        self.imu_msgs_w_covariance = []
        self.gps_msgs_w_covariance = [] 

        print ("Collecting data. Ctrl-C to stop collection and compute covariances")
        rospy.spin()

        fileout = open("/opt/robot/covariances/" + str(str(time.time()) + '__' + self.output_file_name), 'w+')

        for calculators in self.odom_calculators:
            self.odom_msgs_w_covariance.append(calculators.get_odom_w_covariance())

        for calculators in self.imu_calculators:
            self.imu_msgs_w_covariance.append(calculators.get_imu_w_covariance())

        for calculators in self.gps_calculators:
            self.gps_msgs_w_covariance.append(calculators.get_gps_w_covariance())

        count = 0 
        for msg in self.odom_msgs_w_covariance:
            fileout.write('\n\n'+ self.odom_topics[count] +':')
            self.print_formated(msg.twist.covariance, fileout)
            count = count + 1 

        count = 0 
        for msg in self.imu_msgs_w_covariance:
            fileout.write("\n\n" + self.imu_topics[count] + ' (linear)' +':')
            self.print_formated(msg.linear_acceleration_covariance, fileout)
            count = count + 1 

        count = 0 
        for msg in self.imu_msgs_w_covariance:
            fileout.write("\n\n" + self.imu_topics[count] + ' (angular)' +':')
            self.print_formated(msg.angular_velocity_covariance, fileout)
            count = count + 1 

        count = 0 
        for msg in self.gps_msgs_w_covariance:
            fileout.write("\n\n" + self.gps_topics[count] + ':')
            self.print_formated(msg.poistion_covariance, fileout)
            count = count + 1 

    def print_formated(self, covariances, file_in):
        for cov in covariances:
            file_in.write(format(cov, '.20f')+',',)

if __name__ == '__main__':
    rospy.init_node("covariance_calculator_node")
    node = CovarianceCalculatorNode()
    node.compute_cov()
