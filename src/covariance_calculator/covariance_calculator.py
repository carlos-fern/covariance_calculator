import numpy
import collections
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

class CovarianceCollector():

    def __init__(self, col_type):
        self.covariance_data = collections.defaultdict(list)
        self.type = col_type

    def insert_imu(self, msg):
            self.covariance_data[0].append(msg.angular_velocity.x)
            self.covariance_data[1].append(msg.angular_velocity.y)
            self.covariance_data[2].append(msg.angular_velocity.z)

            self.covariance_data[0].append(msg.linear_acceleration.x)
            self.covariance_data[1].append(msg.linear_acceleration.y)
            self.covariance_data[2].append(msg.linear_acceleration.z)

    def insert_odom(self, msg):
        self.covariance_data[0].append(msg.twist.twist.linear.x)
        self.covariance_data[1].append(msg.twist.twist.linear.y)
        self.covariance_data[2].append(msg.twist.twist.linear.z)

        self.covariance_data[3].append(msg.twist.twist.angular.x)
        self.covariance_data[4].append(msg.twist.twist.angular.y)
        self.covariance_data[5].append(msg.twist.twist.angular.z)
    
    def insert_gps(self, msg):
        self.covariance_data[0].append(msg.latitude)
        self.covariance_data[1].append(msg.longitude)
        self.covariance_data[2].append(msg.altitude)

    def get_sample_points(self):
        return self.covariance_data
    
    def get_sample_count(self):
        return length(self.covariance_data[0])

class CovarianceCalculator(CovarianceCollector):
    
    def compute_covariance(self, measure_in, cov_out):
        cov_index = 0
        for axis in measure_in.itervalues():
            for inner_axis in measure_in.itervalues():
                cov_out[cov_index] = self._compute_single_cov(axis, inner_axis)
                cov_index += 1

    def compute_single_covariance(self, x, y):
        sumd = 0
        x_mean = numpy.mean(x)
        y_mean = numpy.mean(y)
        for sample_x, sample_y in zip(x, y):
            sumd += (sample_x - x_mean) * (sample_y - y_mean)
        cov = sumd / (len(x) - 1)
        return cov

    def get_imu_w_covariance():
        imu_msg = Imu()
        self.compute_covariance(self.get_sample_points(), imu_msg.linear_acceleration_covariance)
        self.compute_covariance(self.get_sample_points(), imu_msg.angular_velcoity_covariance)
        return imu_msg

    def get_odom_w_covariance():
        odom_msg = Odometry()
        self.compute_covariance(self.get_sample_points(), odom_msg.twist.covariance)
        return odom_msg

    def get_gps_w_covariance():
        gps_msg = NavSatFix()
        self.compute_covariance(self.get_sample_points(), gps_msg.position_covariance)
        return gps_msg
    


    


