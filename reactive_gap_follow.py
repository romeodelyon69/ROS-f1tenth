#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

pi = 3.14159
close_point = True
buble = False

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        realCar = False

        if(realCar):
             lidarscan_topic = '/scan'
             drive_topic = "/vesc/ackermann_cmd_mux/input/navigation"
        else:
             lidarscan_topic = '/scan'
             drive_topic = '/nav'
	
	
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

        self.cone_angle = 45/180 * pi
    
    def preprocess_lidar(self, ranges, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = []

        indice_inf = int((-self.cone_angle - data.angle_min)*data.angle_increment)
        indice_sup = int((self.cone_angle - data.angle_min)*data.angle_increment)

        rospy.loginfo_throttle(0.5," indice inf : " + str(indice_inf) + "indice sup : " + str(indice_sup))
        
        for dist in ranges[indice_inf : indice_sup]:
            proc_ranges.append(dist)

        proc_ranges = ranges
        return list(proc_ranges)
    
    def buble(self, buble_size, ranges):
        indice = np.argmin(ranges)
        indice_inf = int(indice - buble_size/2)
        indice_sup = int(indice + buble_size/2)

        ranges = list(ranges)

        for i in range(indice_inf, indice_sup + 1):
            ranges[i] = 0
        
        return ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        start_index= 0
        end_index = 0
        max_length = 0

        current_length = 0
        current_start_index = 0

        
        for i, dist in enumerate(free_space_ranges):
            if(dist != 0):
                current_length += 1
            else:
                if(current_length > max_length):
                    print(current_start_index, i)
                    start_index = current_start_index
                    end_index = i-1
                    max_length = current_length

                current_length = 0
                current_start_index = i+1
                
        
        if(current_length > max_length):
            print(current_start_index, i)
            start_index = current_start_index
            end_index = i
            max_length = current_length

        return start_index, end_index
                
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        return None
    
    def remove_close_points(self, threshold, ranges):
        for i,dist in enumerate(ranges):
            if(dist < threshold):
                ranges[i] = 0

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        angle_min = data.angle_min
        incr = data.angle_increment

        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges, data)

        closest_dist = min(ranges)

        #print(proc_ranges)

        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 
        indice_inf = int((-self.cone_angle - data.angle_min)*data.angle_increment)

        if(close_point):
            threshold = close_point * 1.5
            threshold = 0
            threshold = max(threshold, 1)
            self.remove_close_points(threshold, proc_ranges)
        elif(buble):
            ranges = self.buble(10, ranges)

        #Find max length gap 

        start, end = self.find_max_gap(proc_ranges)
        #Find the best point in the gap 
        best_point = int((end+start)/2)

        #Publish Drive message
        velocity = min(proc_ranges[best_point]/2, 7) 
        angle = angle_min + incr * (best_point + indice_inf)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        rospy.loginfo_throttle(0.5,"start : " + str(start) + "end : " + str(end) + "consigne angle : " + str(angle) + "consigne vitesse : " + str(velocity))
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)



def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
