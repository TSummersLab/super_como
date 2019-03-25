#!/usr/bin/env python
'''
We use a checkerboard to get a top-down view of the line.
We then publish the reconstructed image of the line and the top part of the image to different topics

Author:
Sleiman Safaoui
Email:
snsafaoui@gmail.com
sxs169833@utdallas.edu
Git:
The-SS

Date:
June 19, 2018
'''
from __future__ import print_function
import numpy as np
import os
import copy
import time

import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from img_processing_functions import *



class imgFetcher:
    '''
    This class obtains an image from the camera bridge
    '''
    def __init__(self):
        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber("/zed/right/image_raw_color", Image, self.img_callback, queue_size = 1)
        self.img_raw = [] # image obtained from callback
        self.img_raw_indicator = -1 # indicator for image -1 if no img, 'gray' if gray image, 'color' if color image
        self.expected_img_mode = -1 # if unset --> -1, if gray image is expected --> 'gray', if color image expected --> 'color'

    def set_expected_img_mode (self, mode):
        '''
        sets the expected image color format (for grayscale mode = 'gray', for color mode = 'color')

        returns true if set, false if not set
        '''
        if ((mode == 'gray') | (mode == 'color')):
            #print('Image mode set')
            self.expected_img_mode = mode
            return True

        #print('Unidentified image mode')
        self.expected_img_mode = -1
        return False

    def img_callback(self, img):
        '''
        callback for getting an image
        stores returned image in self.img_raw and sets the image indicator self.img_raw_indicator
        '''
        if self.expected_img_mode == -1: # not expecting an image
            self.img_raw = []
            self.img_raw_indicator = -1
            #print('Not expecting an image')
        elif self.expected_img_mode == 'color': # expecting a color image
            try:
                self.img_raw = self.bridge.imgmsg_to_cv2(img, "bgr8")
                self.img_raw_indicator = 'color'
                #print('Received color image')
            except cvBridgeError as e:
                self.img_raw = []
                self.img_raw_indicator = -1
                print (e)
        elif self.expected_img_mode == 'gray': # expecting a grayscale image
            try:
                self.img_raw = self.bridge.imgmsg_to_cv2(img, "mono8")
                self.img_raw_indicator = 'gray'
                #print('Received gray image')
            except cvBridgeError as e:
                self.img_raw = []
                self.img_raw_indicator = -1
                print (e)
        else: # something weird is going on
            self.img_raw = []
            self.img_raw_indicator = -1
            #print('Something weird is going on')
        return

    def get_img_raw_indicator(self):
        '''
        returns the image indicator
        '''
        return self.img_raw_indicator

    def get_img_raw(self):
        '''
        returns the image
        '''
        return self.img_raw



class imgDivider:
    '''
    This class operates on an image and divides it into two parts: a bottom part with the line in it, line_img, and a top part with the rest of the image in it, forward_img.
    '''
    def __init__(self):
        self.img_raw = [] # raw image, to be passed to the object of this class
        self.img_raw_indicator = -1 # indicator for raw image -1 if no img, 'gray' if gray image, 'color' if color image
        self.line_img = [] # bottom part of the image containing the line
        self.line_img_indicator = -1 # # indicator for line image -1 if no img, 'gray' if gray image, 'color' if color image
        self.forward_img = [] # top part of the image containing the image in front of the car
        self.forward_img_indicator = -1 # # indicator for forward image -1 if no img, 'gray' if gray image, 'color' if color image

        self.num_horizontal_corners = -1 # number of checkerboard horizontal corners
        self.num_vertical_corners = -1 # number of checkerboard vertical corners
        self.checkerboard_setup_correct = False # check to see if checkerboard size data imported to self is correct or not (False --> no, True --> yes)
        self.load_corners_check = -1 # check whether to load corners from external file or not (False --> No, True--> Yes)
        self.path_to_corners = "" # path to file containing the outer corners
        self.data_in_file = [] # data in corners.txt file
        self.outer_corners = [] # outer corners of a checkerboard
        self.found_outer_corners = -1 # check whether the outer corners have been found or not (False --> not found, True --> found)
        self.expanded_corners = [] # expanded corners of the checkerboard
        self.found_expanded_corners = False # check whether the expanded corners have been found or not (True --> found, False --> not found )

    def import_img_raw(self, img_raw, img_raw_indicator):
        '''
        imports raw img and its indicator
        returns nothing
        '''
        if ((img_raw_indicator != 'color') & (img_raw_indicator != 'gray')): # no image
            #print('No valid img_raw_indicator')
            self.img_raw = []
            self.img_raw_indicator = -1
        elif img_raw == []: #img_raw_indicator is valid but no image was imported
            #print('No img')
            self.img_raw = []
            self.img_raw_indicator = -1
        else:
            #print('Img delivered')
            self.img_raw = img_raw
            self.img_raw_indicator = img_raw_indicator
        return

    def set_checkerboard_size(self, horizontal, vertical):
        '''
        gets the horizonal and vertical internal corners of a checkerboard and stores that data in self
        returns true if set, false if not set
        '''
        if horizontal < 2:
            #print('Invalid checkerboard size')
            self.checkerboard_setup_correct = False
            return False

        if horizontal == vertical:
            #print('Checkerboard cannot be square')
            self.checkerboard_setup_correct = False
            return False

        if vertical > horizontal:
            #print('Vertical and horizontal dimentions appear to be incorrect. They will be swapped')
            self.num_horizontal_corners = vertical # number of checkerboard horizontal corners
            self.num_vertical_corners = horizontal # number of checkerboard vertical corners
            self.checkerboard_setup_correct = True
            return True

        self.num_horizontal_corners = horizontal
        self.num_vertical_corners = vertical
        self.checkerboard_setup_correct = True
        return True

    def set_path_to_corners(self, path):
        '''
        sets the path to corners.txt file.
        returns true if set, false if not set
        '''
        if path == "":
            #print('No path loaded')
            return False
        self.path_to_corners = path
        return True

    def load_cornersTXT_data(self):
        '''
        loads data from corners.txt and updates whether the data from the file will be used or not
        returns true if file is loaded (or decision was made not to load), false if it could not be loaded
        '''
        if self.path_to_corners == "": # no path added to self
            #print('No path added')
            return False

        data = import_file(self.path_to_corners + '/corners.txt') # import data for corners.txt
        #print('Data Loaded:\n', data)

        if len(data) != 5:
            #print('File does not contain correct data. Data will be overwritten')
            self.load_corners_check = False
            self.data_in_file = []
            return True

        if ((data[0][0] < 0.5) | (data[0][1] < 0.5)): # do not load data
            #print('File indicates that corner data cannot be used. Data will be overwritten by new data')
            self.load_corners_check = False
            self.data_in_file = []
            return True

        #print('Importing data from file')
        self.load_corners_check = True # take corners from data in file
        self.data_in_file = data # data is a list of lists. 5*2

        return True

    def update_cornersTXT_data(self, outer_corners):
        '''
        update corners.txt to store the outer corners
        returns nothing
        '''
        data_write = [[1., 1.]]
        for i in range(4):
            data_write.append(outer_corners[i])
        data_write = np.array(data_write)
        export_file(self.path_to_corners + '/corners.txt', data_write) # overwrite the file containing the corners to store the new corners
        return

    def populate_outer_corners_from_file(self):
        '''
        finds the outer corners of the checkerboard using data imported from the text file
        returns false if population failed, true is successful
        '''
        if self.data_in_file == []:
            #print('No data imported yet')
            return False

        if self.found_outer_corners == True:
            #print('Outer corners already found')
            return True

        # import corners from data
        outer_corners = self.data_in_file[1:5][:]
        #print("Loaded corners:\n", outer_corners)
        self.outer_corners = outer_corners
        self.found_outer_corners = True
        return True

    def populate_outer_corners_from_image(self):
        '''
        finds the outer corners of the checkerboard using the raw image in self and updates the text file
        '''
        if ((self.img_raw_indicator == -1) | (self.img_raw == [])):
            #print('No raw image to find corners')
            return False

        if self.found_outer_corners == True:
            #print('Outer corners already found')
            return True

        all_corners = get_checkerboard_corners(self.img_raw, self.num_horizontal_corners, self.num_vertical_corners) # find all checkerboard corners

        if all_corners == [] : # no corners detected
            #print('No corners detected. Place checkerboard in image frame')
            time.sleep(2) # sleep for 2 seconds
            return False
        # checkerboard corners found
        outer_corners = select_checkerboard_points(all_corners, self.num_horizontal_corners, self.num_vertical_corners) # select outer corners
        self.outer_corners = outer_corners
        self.found_outer_corners = True

        self.update_cornersTXT_data(outer_corners)

        return True

    def find_expanded_corners(self):
        '''
        uses the values of the outer corners in self to calculate the extended corners
        '''
        outer_corners = self.outer_corners
        (cb_bottom_left, cb_bottom_right, cb_top_right, cb_top_left) = outer_corners
        bottom_corner_to_corner = cb_bottom_right[0] - cb_bottom_left[0] # distance between the bottom outer corners of the checkerboard
        if self.img_raw_indicator == 'gray': # gray image
            max_y, max_x = self.img_raw.shape
        elif self.img_raw_indicator == 'color': # color image
            max_y, max_x, channels = self.img_raw.shape
        else: # this should not happen
            #print("Something is worng. Could not find size of image")
            max_x = 0
            max_y = 0
        if ((max_x == 0) | (max_y == 0)): # in case image size cannot be found, use outer corners as extended corners

            self.expanded_corners = outer_corners
            self.found_expanded_corners = True
        else: # extend corners
            self.expanded_corners = extend_corners(max_x, max_y, outer_corners)
            self.found_expanded_corners = True
            #print("expanding corners") #@!
            #print(max_x,max_y,outer_corners) #@!
            #print("0,0", self.img_raw[0][0])
            #print("300,0", self.img_raw[300][0])
            #print("0,300", self.img_raw[0][300])
        return

    def divide_img(self):
        '''
        Divides image raw using the expanded corners (both in self) and stores resulting images in self
        '''
        expanded_corners = self.expanded_corners
        #print (expanded_corners)
        if self.img_raw_indicator == 'gray': # gray image
            max_y, max_x = self.img_raw.shape
        elif self.img_raw_indicator == 'color': # color image
            max_y, max_x, channels = self.img_raw.shape
        else: # this should not happen
            #print("Something is worng. Could not find size of image")
            max_x = 0
            max_y = 0

        # image polygon which is the rectangle in the world frame
        # top, bottom, right, and left are in reference to the viewer
        (bottom_left, bottom_right, top_right, top_left) = expanded_corners

        # compute width of the rectangle to be reconstructed (max length of top or bottom side of the polygon)
        width1 = ( (bottom_right[0] - bottom_left[0])**2 + (bottom_right[1] - bottom_left[1])**2 ) ** 0.5
        width2 = ( (top_right[0] - top_left[0])**2 + (top_right[1] - top_left[1])**2 ) ** 0.5
        width_reconstructed = int(max(width1, width2))

        # compute height of the rectangle to be reconstructed (max length of right or left side of the polygon)
        height1 = ( (top_left[0] - bottom_left[0])**2 + (top_left[1] - bottom_left[1])**2 ) ** 0.5
        height2 = ( (top_right[0] - bottom_right[0])**2 + (top_right[1] - bottom_right[1])**2 ) ** 0.5
        height_reconstructed = int(max(height1, height2))

        reconstructed_rect = np.array([
        [0, height_reconstructed-1],
        [width_reconstructed-1, height_reconstructed-1],
        [width_reconstructed-1, 0],
        [0,0]],
        dtype="float32")

        # compute perspective transform
        transform = cv2.getPerspectiveTransform(expanded_corners, reconstructed_rect)

        #print('Finding line image')
        # transform image image
        self.line_img = cv2.warpPerspective(self.img_raw, transform, (width_reconstructed, height_reconstructed))
        self.line_img_indicator = self.img_raw_indicator
        #print('Found line image')

        y_cutoff = int(min(top_left[1], top_right[1]))

        #print('Finding forward image')
        if self.img_raw_indicator == 'gray':
             self.forward_img = np.array(self.img_raw[0:y_cutoff, 0:max_x]) # gray image
             self.forward_img_indicator = self.img_raw_indicator
             #print('Found forward image')
        elif self.img_raw_indicator == 'color':
            self.forward_img = np.array(self.img_raw[0:y_cutoff, 0:max_x, :]) # color image
            self.forward_img_indicator = self.img_raw_indicator
            #print('Found forward image')
        else:
            self.forward_img = []
            self.forward_img_indicator = -1
            #print('No forward image to be found')

        return

    def get_line_img(self):
        '''
        returns the line image
        '''
        return self.line_img

    def get_line_img_indicator(self):
        '''
        returns the line image indicator
        '''
        return self.line_img_indicator

    def get_forward_img(self):
        '''
        returns the forward image
        '''
        return self.forward_img

    def get_forward_img_indicator(self):
        '''
        returns the forward image indicator
        '''
        return self.forward_img_indicator

    def get_corner_origin_decision(self):
        '''
        returns the dicision about whether to load corners from file or generate them from image
        returns 'file' for load, 'img' for generation, and -1 is still undecided
        '''
        decision = self.load_corners_check
        if decision == -1:
            #print('Undecided')
            return -1

        if decision == True:
            #print('Corners will be loaded')
            return 'file'

        if decision == False:
            #print('Corners should be calculated')
            return 'img'

        return -1

    def get_whether_outer_corners_were_found(self):
        '''
        returns whether or not the outer corners have been found
        '''
        if self.found_outer_corners == True:
            return True
        else:
            return False

    def get_if_outer_corners_were_expanded(self):
        '''
        returns true if they were expaneded and false otherwise
        '''
        if self.found_expanded_corners == True:
            return True
        else:
            return False



class imgPublisher:
    '''
    Publishes the forward image and line image to ROS topics
    '''
    def __init__(self):
        self.bridge = CvBridge()

        self.line_img = [] # bottom part of the image containing the line
        self.line_img_indicator = -1 # # indicator for line image -1 if no img, 'gray' if gray image, 'color' if color image
        self.forward_img = [] # top part of the image containing the image in front of the car
        self.forward_img_indicator = -1 # # indicator for forward image -1 if no img, 'gray' if gray image, 'color' if color image

        self.line_img_pub = rospy.Publisher("/cam/line_img", Image, queue_size = 1)
        self.forward_img_pub = rospy.Publisher("/cam/forward_img", Image, queue_size = 1)

    def import_line_img(self, img, indicator):
        '''
        imports an image to be published to /cam/line_img. also sets the indicator
        returns nothing
        '''
        if indicator == -1:
            #print('Img does not have a color mode. Img will be set to []')
            self.line_img = []
            self.line_img_indicator = -1
        else:
            #print('Importing image')
            self.line_img = img
            self.line_img_indicator = indicator
        return

    def import_forward_img(self, img, indicator):
        '''
        imports an image to be published to /cam/forward_img. also sets the indicator
        returns nothing
        '''
        if indicator == -1:
            #print('Img does not have a color mode. Img will be set to []')
            self.forward_img = []
            self.forward_img_indicator = -1
        else:
            #print('Importing image')
            self.forward_img = img
            self.forward_img_indicator = indicator
        return

    def publish_line_img(self):
        '''
        publishes the content of self.line_img to /cam/line_img if the image is not empty
        '''
        if self.line_img_indicator == -1:
            print('No image to publish')
        elif self.line_img_indicator == 'gray':
            try:
                self.line_img_pub.publish(self.bridge.cv2_to_imgmsg(self.line_img, 'mono8'))
                #print('Line image published')
            except CvBrdigeError as e:
                print(e)
        elif self.line_img_indicator == 'color':
            try:
                self.line_img_pub.publish(self.bridge.cv2_to_imgmsg(self.line_img, 'bgr8'))
                #print('Line image published')
            except CvBrdigeError as e:
                print(e)
        else:
            print('Something is wrong with line image indicator')

        return

    def publish_forward_img(self):
        '''
        publishes the content of self.forward_img to /cam/forward_img if the image is not empty
        '''
        if self.forward_img_indicator == -1:
            print('No image to publish')
        elif self.forward_img_indicator == 'gray':
            try:
                self.forward_img_pub.publish(self.bridge.cv2_to_imgmsg(self.forward_img, 'mono8'))
                #print('Forward image published')
            except CvBrdigeError as e:
                print(e)
        elif self.forward_img_indicator == 'color':
            try:
                self.forward_img_pub.publish(self.bridge.cv2_to_imgmsg(self.forward_img, 'bgr8'))
                #print('Forward image published')
            except CvBrdigeError as e:
                print(e)
        else:
            print('Something is wrong with line image indicator')

        return




def main():
    rospy.init_node("img_preprocessing", anonymous=True)
    rate = rospy.Rate(30)

    # load parameters from ros parameter server
    nodename = "/img_preprocessing"

    num_checkerboard_horizontal_corners = rospy.get_param(nodename + "/CheckerboardParams" + "/num_internal_checkerboard_corners_horizontally")
    num_checkerboard_vertical_corners = rospy.get_param(nodename + "/CheckerboardParams" + "/num_internal_checkerboard_corners_vertically")
    #path = rospy.get_param(nodename + "/path_to_corners")
    #rospy.logfatal('path %s', path)
    #path = "/home/odroid/catkin_ws/src/como_image_processing/config"
    path = "/home/nvidia/super_como/workspace/src/image_processing/config"
    img_mode = rospy.get_param('/img_preprocessing' + "/imgMode")

    # create objects
    fetcher = imgFetcher()
    divider = imgDivider()
    publisher = imgPublisher()

    # initialize fetcher
    set_expected_img_mode_check = fetcher.set_expected_img_mode(img_mode) # set image mode
    if set_expected_img_mode_check == False: # could not set image mode
        #print('Image mode incorrect. Shutting down.')
        return

    # initialize divider
    set_checkerboard_size_check = divider.set_checkerboard_size(num_checkerboard_horizontal_corners, num_checkerboard_vertical_corners) # set checkerboard size
    if set_checkerboard_size_check == False: # could not set image mode
        #print('checkerboard size incorrect. Shutting down.')
        rospy.logfatal('Checkerboard size incorrect')
        return
    set_path_to_corners_check = divider.set_path_to_corners(path) # set path to corners.txt
    if set_path_to_corners_check == False: # could not set path to file
        #print('Incorrect path to corners.txt. Shutting down.')
        rospy.logfatal('Incorrect path to corners.txt')
        return
    load_cornersTXT_data_check = divider.load_cornersTXT_data() # load data from corners.txt
    if load_cornersTXT_data_check == False:
        #print('Incorrect path to corners.txt. Shutting down.')
        rospy.logerr('Failed to load data from corners.txt')
        return
    decision = divider.get_corner_origin_decision() # get decision on whether to load or calculate corners
    if decision == -1: # no decision could be made even though all necesary steps were taken
        rospy.logwarn('Decision about loading corners or calculating them has not been made')
        #print('No decision made. Shutting down.')
        return
    if decision == 'file':
        populate_outer_corners_from_file_check = divider.populate_outer_corners_from_file()
        if populate_outer_corners_from_file_check == False:
            #print('Failed to load corners correctly. Shutting down') # could not load corners even though all necesary steps were taken
            rospy.logerr('Failed to load corners corretly')
            return
    # if decision was to calculate new corners, that cannot be started yet

    # start main image preprocessing loop
    try:
        while not rospy.is_shutdown():
            #####Pseudo code for section#####
            # get a new image
            # send image to divider
            # if still not calculated, find corners from image
            # if corners are available and still not extended, extend them
            # divide the image
            # send divided images to publisher
            # publish images
            #####END#####

            # get a new image
            img_raw = fetcher.get_img_raw()
            img_raw_indicator = fetcher.get_img_raw_indicator()
 
            #if img_raw_indicator != -1:
            if ((img_raw_indicator == -1) | (img_raw == [])): # there is no image
                rospy.logwarn("No image yet.")
            else: # image exists
                # send image to divider
                divider.import_img_raw(img_raw, img_raw_indicator) # import raw image to divider
                outer_corners_exist_check = divider.get_whether_outer_corners_were_found() # check if outer corners exist
                if (outer_corners_exist_check == False): # corners not found yet
                    # find corners from image
                    populate_outer_corners_from_image_check = divider.populate_outer_corners_from_image() # get outer if there is need

                else: # corners exist, check if they have been expanded
                    # check if corners have been expanded
                    if (divider.get_if_outer_corners_were_expanded() == False): # corners have not been expanded
                        # if corners are available and still not extended, extend them
                        divider.find_expanded_corners()
                    else: # corners are found and image is found
                        # divide the image
                        divider.divide_img()
                        line_img = divider.get_line_img()
                        line_img_indicator = divider.get_line_img_indicator()
                        forward_img = divider.get_forward_img()
                        forward_img_indicator = divider.get_forward_img_indicator()

                        # send divided images to publisher
                        publisher.import_line_img(line_img, line_img_indicator)
                        publisher.import_forward_img(forward_img, forward_img_indicator)

                        # publish images
                        publisher.publish_line_img()
                        publisher.publish_forward_img()
                        #print("Images published")

            rate.sleep()

    except KeyboardInterrupt:
        print("Shutting down")
        rospy.logfatal("Keyboard Interrupt. Shutting down image_preprocessing node")
    #cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down image_preprocessing node")
        pass
