#!/usr/bin/env python
'''
store rosbag data into videos and csv files

AUthor:
Sleiman Safaoui
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu
GitHub:
The-SS

Date:
Feb 25, 2019

'''
from __future__ import print_function

import numpy as np
from numpy import pi
import time
import csv

import rospy 
import roslib
import rosbag

from image_processing.msg import LineData



class lineDataSub:
	'''
	Fetches data published to /line_data 
	'''
	def __init__(self):
		self.data_sub = rospy.Subscriber("/line_data", LineData, self.data_callback, queue_size = 1)
		self.line_data = []

	def data_callback(self, linedata):
		self.line_data = linedata
		return

def store_data_csv(file_name, data, msg):
    '''
    Writes data to a file called file_name.csv (overwrites previous data)
    input:
            msg: string containing a message to be printed on the first line only
            file_name: string indicating file name
            data: list containing the data
    output:
            True if successful, False if not successful
    '''
    if file_name == "":
        print('Invalid file name')
        return False

    full_file_name = file_name + '.csv'

    with open(full_file_name, 'w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=",")
        csv_writer.writerow([msg])
        for i in range(len(data)):
            csv_writer.writerow(data[i])

    return True


def main():
    rospy.init_node("data_storage") # init ros node
    num_files = 35
    for idx in range(num_files+1):
        if idx == 0:
            continue
        bag_name = "/home/nvidia/linedata_04122019/run_st" + str(idx) #bag file name and path 
        bag = rosbag.Bag(bag_name+".bag") # create an object for the bag file
    
        T= []
        line_data = []
        for topic, msg, t in bag.read_messages(topics=["/line_data"]):
            ts = t.to_sec()
            T.append(t.to_sec())
            line_data.append([ts, msg.angle_radian, msg.angle_degree, msg.line_pos_x, msg.line_pos_y])
        #print(len(T))
        #print(len(line_data))

        bag.close()
    
    
    	    
        print("Storing data for ", idx)
        save_file_name = bag_name
        line_data_msg = "Line Data. Each row has: timestamp (sec), line angle (radian), line_angle (degree), line x pos (meter), line y pos (meter)"
   
        store_data_csv(file_name = save_file_name + "_line_data", data = line_data, msg = line_data_msg)
 
    print("done")


if __name__=="__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		rospy.logfatal("ROS Interrupt. Shutting down data storage")
		pass

