#!/usr/bin/env python
'''
This code uses the Hokuyo UST-10LX Lidar data obtained thru the racecar package thru scomo.launch to drive the car along a wall to its left. Along with Object Detection from Darknet's YOLO package.

Date:
April 4, 2019
'''
# Python imports
from __future__ import print_function
import numpy as np
import time
from numpy import pi

#ROS Imports
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped as ADS

#Objects detected by ZED
objects = ["stop sign"]
stoptime = 200

#thresholds for wall tracing
front_thresh = .87
right_thresh = 1.
left_thresh = 2.
PID_thresh = 0.61
reverse_dist = 0.1

default_forward = 1.2
default_turn = 1.
MAX_servo = -0.34
MIN_servo = 0.34

Kp = 1.5 #3
Ki = 0.
Kd = 0.0

i_max= 10.0
i_min= -10.0
ctrl_max= 0.34
ctrl_min= -0.34

col_ang = 23. # angle with respect to heading to scan for potential collisions (scan will be for [-col_ang, +col_ang]
abs_stop_thresh = 0.25 # min distance to obstacle ahead below which the car would stop

max_lidar_range = 5.0

class PID_ctrl:
    
    def __init__(self, Kp = 1., Ki = 0., Kd = 0., i_max = 0., i_min = 0., ctrl_max = 1., ctrl_min = 0.):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.i_max = i_max
        self.i_min = i_min
        self.ctrl_max = ctrl_max
        self.ctrl_min = ctrl_min
        self.e_curr = 0.
        self.e_prev = 0.
        self.e_sum = 0.
        self.t_curr = 0.
        self.t_prev = 0.

    def apply_pid(self, des_value, current_value, timestamp):
        self.e_curr = des_value - current_value
        print('error', self.e_curr)
        self.t_curr = timestamp
        dt = self.t_curr - self.t_prev
        #print('dt', dt)

        # Proportional control
        p_ctrl = self.Kp * self.e_curr
        print('p_ctrl', p_ctrl)

        # Integral control with anti-windup
        i_ctrl = self.e_sum + self.Ki*dt/2.0*(self.e_curr+self.e_prev)
        i_ctrl = min(i_ctrl, self.i_max)
        i_ctrl = max(i_ctrl, self.i_min)
        #print('i_ctrl', i_ctrl)

        # Derivative control
        d_ctrl = self.Kd*(self.e_curr-self.e_prev)/dt
        #print('d_ctrl', d_ctrl)

        # Control signal calculation
        ctrl = p_ctrl + i_ctrl + d_ctrl

        # Control saturation
        ctrl = min(ctrl, self.ctrl_max)
        ctrl = max(ctrl, self.ctrl_min)

        # update previous values with current values
        self.e_sum = i_ctrl
        self.e_prev = self.e_curr
        self.t_prev = self.t_curr

        return ctrl

class LidarFetcher:
    '''
    Fetches data published to /lidar_data 
    '''
    def __init__(self):
        self.data_sub = rospy.Subscriber("/scan", LaserScan, self.data_callback, queue_size = 1)
        self.lidar_data = []

    def data_callback(self, lidardata):
        self.lidar_data = lidardata
        return
        
    def get_lidardata(self):
        return self.lidar_data


def goBack(init_front_dist, rev_dist, rate, ads_msg, fetcher, publisher):
    servo = 0.
    motor = 0.
    for i in range(20):
        timestamp = time.time()
        ads_msg.header.seq+=1
        ads_msg.header.stamp.secs = timestamp
        ads_msg.drive.steering_angle = 0
        ads_msg.drive.speed = 0
        
        publisher.pub_vesc_msg(ads_msg) # publish the message
        rate.sleep() # sleep until next iteration
        
    data = fetcher.get_lidardata()
    front_dist_now = data.ranges[540]
        
    while front_dist_now < init_front_dist + rev_dist:
        print("Going back")
        print('Dist now', front_dist_now)
        print('init Dist', init_front_dist)
        timestamp = time.time()
        data = fetcher.get_lidardata()
        front_dist_now = data.ranges[540]
        
        # go in reverse
        motor = - default_forward / 2.0
        servo = 0
        
        ads_msg.header.seq+=1
        ads_msg.header.stamp.secs = timestamp
        ads_msg.drive.steering_angle = servo
        ads_msg.drive.speed = motor
        
        publisher.pub_vesc_msg(ads_msg) # publish the message
        rate.sleep() # sleep until next iteration
    
    for i in range(20):
        timestamp = time.time()
        ads_msg.header.seq+=1
        ads_msg.header.stamp.secs = timestamp
        ads_msg.drive.steering_angle = 0
        ads_msg.drive.speed = 0
        
        publisher.pub_vesc_msg(ads_msg) # publish the message
        rate.sleep() # sleep until next iteration
        
    data = fetcher.get_lidardata()
    front_dist_now = data.ranges[540]
        
class VESCPublisher:
    def __init__(self):
        self.VESC_msg = ADS()
        self.VESC_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", ADS, queue_size = 1)
    
    def set_VESC_msg(self, ads_msg):
        '''
        Sets vesc ADS msg
        '''
        self.ADS = ads_msg
        
    def pub_vesc_msg(self, ads_msg):
        '''
        Sets vesc ADS msg and publishes it
        '''
        self.set_VESC_msg(ads_msg)
        self.VESC_pub.publish(self.ADS)   



class ZEDFetcher:
    '''
    Fetches data published to /lidar_data 
    '''
    def __init__(self):
        self.data_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.data_callback, queue_size = 1)
        self.zed_data = []

    def data_callback(self, zeddata):
        self.zed_data = zeddata
        return
        
    def get_zed_data(self):
        return self.zed_data

def main():
    rospy.init_node("HallNavwithObjDet")
    rate = rospy.Rate(40)
    servo = 0.
    motor = 0.
    
    control = PID_ctrl(Kp = Kp, Ki = Ki, Kd = Kd, \
                    i_max = i_max, i_min = i_min, \
                    ctrl_max = ctrl_max, ctrl_min = ctrl_min) # PID controller 
    fetcher = LidarFetcher() # gets lidar data
    publisher = VESCPublisher() # publishes control command to VESC                        
    
    lidardata = fetcher.get_lidardata()
    while lidardata == []: # while lidar data has not been populated
        print("Waiting for Lidar data to populate")
        lidardata = fetcher.get_lidardata() # update lidardata
        
    # get lidar characteristics
    ang_min = lidardata.angle_min / pi * 180 # minimum lidar scan angle in degrees
    ang_max = lidardata.angle_max / pi * 180 # maximum lidar scan angle in degrees
    ang_step = lidardata.angle_increment / pi *180 # lidar angle increment
    
    zedfetcher = ZEDFetcher()
    detection = zedfetcher.get_zed_data()
    timestamp = time.time()
    detectedtime = timestamp
    
    while not rospy.is_shutdown():
        timestamp = time.time()
        detection = zedfetcher.get_zed_data()
        detected = False
        if detection == []: # check if lidar data is populated
            continue
            
        for i in detection.bounding_boxes:
            if (i.Class in objects):
                print("DETECTED ", i.Class, "at ", timestamp)
                detected = True
                break
        
        ads_msg = ADS()
        lidardata = fetcher.get_lidardata() # get lidar data
        if lidardata == []: # check if lidar data is populated
            continue # if not populated --> skip to next iteration

        right_dist = lidardata.ranges[180] # extract right distance
        front_dist = lidardata.ranges[int(540 - col_ang*4):int(540 + col_ang*4 + 1)] # extract a range of front measurements 
        left_dist = lidardata.ranges[900] # extract left distance
        
        print('right', right_dist)
        print('front', front_dist[int(len(front_dist)/2)])
        print('left', left_dist)
        #print('all', type(lidardata.ranges))
        
        # check if any of the front distance measurements are below a certain threshold
        front_blocked = False  
        min_front = min(front_dist)     

        print('min_front', min_front)
        
        if min_front < front_thresh:
            front_blocked = True
                
        if min_front < abs_stop_thresh:
            #motor = 0
            if lidardata.ranges[540] < max_lidar_range:
                goBack(init_front_dist = lidardata.ranges[540], rev_dist= reverse_dist, rate = rate, \
                        ads_msg = ads_msg, fetcher = fetcher, publisher = publisher)
            print("Too close to obstacle ahead. Cannot move")
            
        elif (detected and (timestamp - detectedtime) > 11.0):
            print("Im not going to drive past the stop sign")
            detectedtime = timestamp
            print("DETECTED at ", timestamp)
            for i in range(stoptime):
                timestamp = time.time()
                ads_msg.header.seq+=1
                ads_msg.header.stamp.secs = timestamp
                ads_msg.drive.steering_angle = 0
                ads_msg.drive.speed = 0
                    
                publisher.pub_vesc_msg(ads_msg) # publish the message
                rate.sleep() # sleep until next iteration  

        elif (front_blocked): # check if front is blocked
            if(left_dist < left_thresh): # check if left is blocked
                if(right_dist < right_thresh): # check if right is blocked
                    # this is a deadend
                    motor = 0.
                    print("deadend")
                else:
                    # blocked ahead but opening on right side (hallway on right side)
                    # sharp right turn
                    motor = default_turn
                    servo = MAX_servo
                    print("rightturn")
            else:
                # blocked ahead but opening on left side (hallway on left side)
                # sharp left turn
                motor = default_turn
                servo = MIN_servo
                print("leftturn")
        else:
            if(left_dist < left_thresh): # check if there is a close wall/object to the left
                # nothing ahead + wall to left
                # regulate left distance through PID
                motor = default_forward
                servo = - control.apply_pid(des_value = PID_thresh, current_value = left_dist, timestamp = timestamp)
                print("PID control")
            else:
                # nothing ahead + opening to the left (hallway on left side)
                # sharp left turn
                motor = default_turn
                servo = MIN_servo
                print("leftturn")
        
        # populate the message to send to the ESC        
        ads_msg.header.seq+=1
        ads_msg.header.stamp.secs = timestamp
        ads_msg.drive.steering_angle = servo
        ads_msg.drive.speed = motor
        
        publisher.pub_vesc_msg(ads_msg) # publish the message
        rate.sleep() # sleep until next iteration
        
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logfatal("ROS Interrupt. Shutting down hallwaynavigation node")
		pass
