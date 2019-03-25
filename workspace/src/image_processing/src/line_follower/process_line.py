#!/usr/bin/env python
'''
We get the top-down view image of the line and process it to find the line's orientation

Author:
Sleiman Safaoui
Email:
snsafaoui@gmail.com
sxs169833@utdallas.edu
Git:
The-SS

Date:
June 21, 2018
'''
from __future__ import print_function
import numpy as np
import os
import copy
from collections import deque

import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from img_processing_functions import *
from image_processing.msg import LineData


'''
Notes:


image indicator: 'gray', 'color', ''
image encoding: 'mono8', 'bgr8', ''
encoding = color_mode_to_encoding(indicator)
indicator = encoding_to_color_mode(encoding)


Reference frames:
Fc: right-handed Camera Frame: the frame that openCV assigns to an image (y down, x right, origin at top-left corner of the image)
Fi: right-handed Image Frame: a frame we assign to the image (y up, x right, origin at bottom-left corner of the image)
Fb: right-handed Body Frame: a frame we assign to the car (y up, x left, origin at the center of the car)


      Oc .------> Xc ---------------------------|
         |                                      |
         V                           C          |      C: blob center
        Yc                                      |
        Yi -----------------R-------------------|      R: Median of side
         ^                                      |
         |                                      |
         . ----> Xi ----------------------------|

                           Yb
                           ^
                   0-------|-------0 ----------------> Front end of the car
                           |
                           |
             Xb <--------- . Ob
                           |
                           |
                   0---------------0 ----------------> Rear end of the car

'''

class imgFetcher:
    '''
    Fetches images from a certain ros topic publishing images
    currently it only supports gray and color images
    '''
    def __init__(self, topic, mode):
        self.bridge = CvBridge() # img bridge
        self.img_sub = rospy.Subscriber(topic, Image, self.img_callback, queue_size = 1) # topic to subscribe to
        self.img = [] # obtained image
        self.img_indicator = '' # indicator for image '' if no img, 'gray' if gray image, 'color' if color image
        # self.img_encoding is the expected encoding
        self.img_encoding = color_mode_to_encoding(mode)
    def set_img_mode(self, mode):
        '''
        sets self.image_mode, in case changes are required
        '''
        self.img_encoding = color_mode_to_encoding(mode)
        return
    def img_callback(self, img):
        '''
        callback for getting an image
        '''
        if self.img_encoding == '':
            #print('Image encoding not defined. Cannot fetch image')
            rospy.logfatal('Image encoding not defined. Cannot fetch image')
            return
        # with valid image mode
        try:
            self.img = self.bridge.imgmsg_to_cv2(img, self.img_encoding)
            self.img_indicator = encoding_to_color_mode(self.img_encoding)
        except cvBridgeError as e:
            self.img = []
            self.img_indicator = -1
            rospy.logerr(e)
            print(e)
        return
    def get_img(self):
        '''
        returns the obtained image
        '''
        return self.img
    def get_img_indicator(self):
        '''
        returns the indicator corresponding to the image
        '''
        return self.img_indicator

class lineProcessor:
    '''
    imports an image and contains functions to process it, detect a line, find the angle of the line, and find the origin of the line.
    Then it can map the obtained data from the camera frame to the car body frame
    '''
    def __init__(self):
        self.bridge = CvBridge() # img bridge
        self.img = [] # image
        self.img_indicator = '' # indicator for image '' if no img, 'gray' if gray image, 'color' if color image
        self.img_gray = [] # grayscale version of the image
        self.img_gray_indicator = '' # indicator for the gray image ('gray' or '')
        self.img_bw = [] # black and white image containing the line
        self.img_edges = [] # edges of img_bw
        self.img_mask = [] # mask containing line only
        self.rows = 0 # rows in image
        self.cols = 0 # columns in image
        self.size_set_check = False # check whether the size of image (rows and cols) has been set or not
        self.img_arrow_color = [] # image with arrow indicating orientation of the line (colored image)
        self.img_arrow_gray = [] # grayscale version of img_arrow_color
        self.center = [] # center of the line in the image (in pixels)
        self.angle = [] # angle of line in the camera frame with the origin in the top-left corner (angle is based on the image) first is the value in radian, second is the value in degrees
        self.angle_line_body = [] # angle of line in the body frame. first is the value in radian, second is value in degrees
        self.car_to_img_center_meters = [] # lateral and vertical distance between the car's body frame origin and the center of the line (in meters)
        self.heading_vector = [] # heading vector of the line
        self.corners_from_file = [] # corners loaded from corners.txt
        self.prev_line_center = [0, 0] #median of the previous line distances
        # imported parameters
        self.param_update = False # if below parameters have not been updated, process_line_img cannot execute
        self.threshold = 0 # threshold to binarize image
        self.width_line = 0.0 # expected width of the line
        self.width_gain = 0.0 # percentage of width of line to use in calculation of minimum size of line
        self.arrow_scale = 0.0 # multiplies to determine the size of the arrow
        # parameters for mapping camera frame to car body frame
        self.num_horizontal_corners = -1 # number of checkerboard horizontal corners
        self.num_vertical_corners = -1 # number of checkerboard vertical corners
        self.square_size = 0.0 # size of a side of a checkerboard square
        self.car_body_frame_to_board = 0.0 # distance between car center and checkerboard center in meters
        self.checkerboard_setup_correct = False # check to see if checkerboard size data imported to self is correct or not (False --> no, True --> yes)
    def set_param(self, threshold, width_line, width_gain, arrow_scale):
        if ((threshold < 0) | (threshold > 255)):
            rospy.logfatal('Invalid threshold')
            self.param_update = False
            return
        if (width_line < 0):
            rospy.logfatal('Invalid line width')
            self.param_update = False
            return
        if (width_gain < 0):
            rospy.logfatal('Invalid width gain')
            self.param_update = False
            return
        if (arrow_scale < 0):
            rospy.logfatal('Invalid arrow scale')
            self.param_update = False
            return
        self.threshold = threshold
        self.width_line = width_line
        self.width_gain = width_gain
        self.arrow_scale = arrow_scale
        self.param_update = True
        return
    def import_img(self, img, indicator):
        '''
        imports raw img and its indicator
        returns true if image was set, false if image not set
        '''
        if indicator == '':
            rospy.logerr('No image indicator')
            #print('No indicator')
            self.img = []
            self.img_indicator = ''
            return False

        if img == []:
            rospy.logerr('No image')
            #print('No image')
            self,img = []
            self.img_indicator = ''
            return False

        #print('Image delivered')
        self.img = img
        self.img_indicator = indicator
        return True
    def color_to_gray(self, img, indicator):
        '''
        takes a colored image as an input and produces a grayscale vesion of it
        returns the gray image and its indicator
        '''

        if indicator == 'gray': # it as already gray
            return img, indicator

        if indicator == 'color': # convert color to gray
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            return img_gray, 'gray'

        return [], ''
    def binarize_img(self, img, threshold):
        '''
        takes an image as input and produces a binary image from it based on the input threshold
        returns the black and white image
        '''
        ret, bw = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY)
        return bw
    def find_edges(self, img):
        '''
        takes an image as input and finds edges in it using a canny edge detector
        returns image with edges
        '''
        edges = cv2.Canny(img, 50, 150, apertureSize = 3)
        return edges
    def find_contours(self, img):
        '''
        takes an image as input and finds contours in it
        returns the contours
        '''
        ret_im, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours
    def vote_for_contours(self, contours, cols, rows, min_area, min_perimeter):
        '''
        takes contours and other variables as inputs and weighs the contours based on their similarity to what is expected of the line
        returns the votes
        '''
        # check the characteristics of each contour and increase its votes if they match the chatacteristics of the line
        votes = np.zeros([len(contours)])
        corner_to_corner = np.linalg.norm(np.array([rows - 1, cols - 1])) # diagonal of image

        i = 0
        for cnt in contours:
            epsilon = 0.05*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True)
            area = cv2.contourArea(cnt) # area of contour
            perimeter = cv2.arcLength(cnt,True) # perimeter of contour
            moments = cv2.moments(cnt) # find the moments of the contour
            if moments['m00'] == 0:
                cx = float('inf')
                cy = float('inf')
            else:
                cx = int(moments['m10']/moments['m00'])# x value for centroid of contour
                cy = int(moments['m01']/moments['m00'])# y value for centroid of contour
            contour_to_prev_line = np.linalg.norm(np.array([cx,cy]) - np.array(self.prev_line_center))

            if (area > min_area): # check if area is greater than min value
                votes[i] += 2.
            if (perimeter > min_perimeter): # check if perimeter is greater than min value
                votes[i] += 2.
            for j in range(len(approx)): # check how many points you have on the top and bottom edges of the image (the line should have 4)
                if (approx[j,0,1] == 0):
                    votes[i] += 1.
                elif approx[j,0,1] == rows-1:
                    votes[i] += 1.
            if len(approx) > 3: # check how many points the contour has (a line is a square --> at least 4 points)
                votes[i] += 2.
            votes[i] += 20.*(1 - contour_to_prev_line/corner_to_corner) # the closer the object is to the previous line, the higher its vote

            i = int(i+1)
        return votes
    def find_idx_of_line_contour(self, num_contours, votes):
        '''
        takes votes for contours as input and returns the index of the contour with the highest score. This index is expected to correspond to the contour of the line.
        returns the index
        '''
        line_idx = 0
        for i in range(num_contours):
            if votes[i] == max(votes):
                line_idx = i
        return line_idx
    def mask_image(self, cols, rows, contour):
        '''
        takes the contour of the line and returns an image containing the line only
        returns the mask as a numpy array
        '''
        mask = np.zeros([rows, cols])
        cv2.drawContours(mask, contour, 0, 255, -1)
        return mask
    def find_line_data(self, mask):
        '''
        takes in an array containing the data corresponding to the line only and returns the center and orientation of the line and the eigen vector
        center is returned as a list containing x and y data
        if found, orientation is returned as a list of floats where first element is angle in rad and the second is in degree. Else, an empty list is returned
        if found, the eigen vector is returned as a list the first is the x component, and the second is the y component. Else, the list is empty
        '''
        y, x = np.nonzero(mask) # get x and y of white pixels
        center_x = np.mean(x)
        center_y = np.mean(y)
        x = x - np.mean(x) # shift x values by their mean
        y = y - np.mean(y) # shift y values by their mean
        coord = np.vstack([x,y]) # stack the coordinates
        cov = np.cov(coord) # find the covarience in the coordinates

        try: # find orientation of line
            eig_vals, eig_vecs = np.linalg.eig(cov)
            sort_indices = np.argsort(eig_vals)
            sort_indices = np.argsort(eig_vals)[::-1] # sort eigenvalues in decreasing order
            x_v1, y_v1 =  - eig_vecs[:, sort_indices[0]]  # Eigenvector with largest eigenvalue
            ang = math.atan2(y_v1, x_v1) # the obtained angle is with respect to the image x-axis
            ang = bound_angle_rad(ang) # bound the angle of the Eigenvector so that it is between 0 and pi radian in the image
            angd = ang/math.pi * 180
            return [center_x, center_y], [ang, angd], [x_v1, y_v1]

        except: # could not find orientation of line
            rospy.logwarn('Failed to find eigenvectors and eigenvalues')
            #print('failed to find eigenvectors and eigenvalues')
            y, x = np.nonzero(mask) # get x and y of white pixels
            return [center_x, center_y], [], []
    def process_line_img(self):
        '''
        processes the image in self to obtain the center and orientation of the line
        it updates center, angle, img_gray, img_gray_indicator, img_bw, img_edges, img_arrow_color and img_arrow_gray in self
        returns true if angle and orientation were found
        returns false otherwise
        '''
        if self.param_update == False:
            rospy.logwarn('Please update parameters first')
            return False
        threshold = self.threshold
        width_line = self.width_line
        width_gain = self.width_gain
        scale = self.arrow_scale
        if self.img == []:
            rospy.logwarn('No image to process')
            #print('No image to process')
            return False
        if self.img_indicator == '':
            rospy.logwarn('Image and indicator not in sync')
            #print('Image and indicator not in sync')
            return False
        # get a gray image from the image
        img_gray, img_gray_indicator = self.color_to_gray(self.img, self.img_indicator)
        # stop if there was a problem with obtaining the gray image
        if img_gray_indicator == '':
            rospy.logerr('Could not generate a gray image')
            #print('Could not get a gray image')
            return False
        self.img_gray = img_gray.copy()
        self.img_gray_indicator = img_gray_indicator

        # binarize the image
        img_bw = self.binarize_img(img_gray, threshold)
        if img_bw == []: # if the black and white image is empty, stop
            rospy.logwarn('No black and white image')
            return False
        self.img_bw = img_bw.copy()

        # find edges
        img_edges = self.find_edges(img_bw)
        if img_edges == []: # if the black and white image is empty, stop
            rospy.logwarn('No edges detected')
            return False
        self.img_edges = img_edges.copy()

        if self.size_set_check == False: # rows and columns have not been found
            rows, cols = img_bw.shape # find size of the image
            if ((rows != 0) & (cols != 0)):
                self.rows = rows
                self.cols = cols
        else: # load size of image if it has already been found
            rows = self.rows
            cols = self.cols

        # calculate thresholds used later in the code
        min_area = rows*(width_line * width_gain)
        min_perimeter = 2.*rows*(width_line * width_gain)

        # find contours
        contours = self.find_contours(img_bw)
        if contours == []:
            rospy.logwarn('No contours detected')
            return False

        img_arrow_gray = self.img_gray.copy() # copy the gray image to draw line on it
        img_arrow_color = cv2.cvtColor(img_arrow_gray, cv2.COLOR_GRAY2BGR) # make a colored version of this image (only orientation of line will be colored)

        # give scores to contours (to determine which one might be the line)
        votes = self.vote_for_contours(contours, cols, rows, min_area, min_perimeter)

        # find the index of the contour with the most votes
        line_idx = self.find_idx_of_line_contour(len(contours), votes)

        # mask the image to get the line only
        mask = self.mask_image(cols, rows, [contours[line_idx]])

        # find the center of the line and its orientation
        center, angle, eigen_vector = self.find_line_data(mask)
        [center_x, center_y] = center
        self.center = [center_x, center_y]
        if angle == []: # could not find angle
            rospy.logwarn('Could not find angle of line')
            return False
        [ang_rad, ang_deg] = angle
        self.angle = [ang_rad, ang_deg]
        [eigen_vector_x, eigen_vector_y] = eigen_vector
        self.heading_vector = [eigen_vector_x, eigen_vector_y]

		#updating prev_line_center
        self.prev_line_center = [int(center_x), int(center_y)] 
        
        # center of blob in top-down-view image
        pt1 = (int(center_x), int(center_y)) #start point (center)
        pt2 = (int(center_x+eigen_vector_x*scale), int(center_y+eigen_vector_y*scale)) #end point (center -
        try:
            cv2.arrowedLine(img_arrow_color, pt1, pt2, (150,130,180), 2)
            self.img_arrow_color = img_arrow_color.copy()
            img_arrow_gray = cv2.cvtColor(img_arrow_color, cv2.COLOR_BGR2GRAY)
            self.img_arrow_gray = img_arrow_gray.copy()
        except: # if an error occures while plotting the line on the image, skip the plot
            pass

        return True
    def set_checkerboard_data(self, horizontal, vertical, square_size, car_body_frame_to_board):
        '''
        gets and stores the following in self:
            size of a checkerboard square
            distance between car and checkerboard
            horizonal and vertical internal corners of a checkerboard
        returns true if set, false if not set
        '''
        if car_body_frame_to_board < 0.0:
            rospy.logfatal('Invalid distance to board')
            #print('Invalid distance to board')
            self.checkerboard_setup_correct = False
            return False

        if square_size < 0.0:
            rospy.logfatal('Invalid square size')
            #print('Invalid square size')
            self.checkerboard_setup_correct = False
            return False

        if horizontal < 2:
            rospy.logfatal('Invalid checkerboard size')
            #print('Invalid checkerboard size')
            self.checkerboard_setup_correct = False
            return False

        if horizontal == vertical:
            rospy.logfatal('Checkerboard cannot be square')
            #print('Checkerboard cannot be square')
            self.checkerboard_setup_correct = False
            return False

        if vertical > horizontal:
            rospy.logwarn('Vertical and horizontal dimentions appear to be incorrect. They will be swapped')
            #print('Vertical and horizontal dimentions appear to be incorrect. They will be swapped')
            self.num_horizontal_corners = vertical # number of checkerboard horizontal corners
            self.num_vertical_corners = horizontal # number of checkerboard vertical corners
            self.car_body_frame_to_board = car_body_frame_to_board
            self.square_size = square_size
            self.checkerboard_setup_correct = True
            return True

        self.num_horizontal_corners = horizontal
        self.num_vertical_corners = vertical
        self.car_body_frame_to_board = car_body_frame_to_board
        self.square_size = square_size
        self.checkerboard_setup_correct = True
        return True
    def load_cornersTXT_data(self, path):
        '''
        loads data from corners.txt
        returns true if file is loaded and false if it could not be loaded/ data invalid
        '''
        if path == "": # no path
            rospy.logfatal('Invalid path to corners.txt')
            #print('Path no valid')
            self.corners_from_file = []
            return False

        data = import_file(path + '/corners.txt') # import data for corners.txt
        #print('Data Loaded:\n', data)

        if len(data) != 5: # incorrect data
            rospy.loginfo('corners.txt does not contain correct data')
            #print('File does not contain correct data.')
            self.corners_from_file = []
            return False

        if ((data[0][0] < 0.5) | (data[0][1] < 0.5)): # do not load data
            rospy.loginfo('File indicates that corner data cannot be used')
            #print('File indicates that corner data cannot be used.')
            self.corners_from_file = []
            return False

        rospy.loginfo('Importing data from file')
        #print('Importing data from file')
        self.corners_from_file = data # data is a list of lists. 5*2

        return True
    def transfor_line_data_to_body(self):
        '''
        applies transformation to line data (center and orientation)
        foo_p: foo in pixels (foo can be anything)
        foo_m: foo in meters (foo can be anything)
        returns true if successful and false if not
        '''
        rows = self.rows
        cols = self.cols
        num_vertical_checkerboard_corners = self.num_vertical_corners
        num_horizontal_checkerboard_corners = self.num_horizontal_corners
        square_size = self.square_size
        car_body_frame_to_board = self.car_body_frame_to_board
        cC_p = self.center
        cHeading_p = self.heading_vector
        corners_from_file = self.corners_from_file

        if corners_from_file == []:
            rospy.logwarn('No outer corners yet')
            #print('No outer corners yet')
            return False
        if ((corners_from_file[0][0] < 0.5) | (corners_from_file[0][1] < 0.5)):
            rospy.logwarn('Corners need update')
            #print('Corners need update')
            return False
        if square_size == 0:
            rospy.logwarn('Square size not set yet')
            #print('Square size not set yet')
            return False
        outer_corners = corners_from_file[1:5][:]
        # mapping between pixels and meters
        num_horizontal_checkerboard_squars = num_horizontal_checkerboard_corners-1 # internal to the corners only
        num_vertical_checkerboard_squars = num_vertical_checkerboard_corners-1 # internal to the corners only
        (cb_bottom_left, cb_bottom_right, cb_top_right, cb_top_left) = outer_corners
        bottom_corner_to_corner = cb_bottom_right[0] - cb_bottom_left[0]
        #print(' bottom_corner_to_corner',  bottom_corner_to_corner)
        #print('SQR SIZE', square_size)
        img_x_ratio_pixles_per_meters = bottom_corner_to_corner / (num_horizontal_checkerboard_squars * square_size) # number of pixels between the bottom two corners of the checkerboard is proportional to the length of that side in meters
        img_y_ratio_pixles_per_meters = rows / (num_vertical_checkerboard_squars * square_size) # the internal squars take up all the height of the image --> ratio = num_rows / (num_sqrs * sqr_size)
        car_body_frame_to_board_p = car_body_frame_to_board * img_y_ratio_pixles_per_meters

        iHc = np.array( [ [1,0,0,0], [0, -1, 0, rows], [0, 0, -1, 0], [0, 0, 0, 1] ] )

        bHi = np.array( [ [-1, 0, 0, cols/2.], [0, 1, 0, car_body_frame_to_board_p], [0, 0, -1, 0], [0, 0, 0, 1] ] ) # transformation matrix: Fi with respect to Fb

        bHc = np.dot(bHi, iHc) # tranformation matrix: Fc with respect to Fb

        # mapping point C to Fb
        bC_p = applyPtTransformation_2D(bHc, cC_p)
        bC_m = [bC_p[0]/img_x_ratio_pixles_per_meters, bC_p[1]/img_y_ratio_pixles_per_meters]
        # mapping heading of line to Fb and finding its angle
        bHeading_p = applyVectorTransformation_2D(bHc, cHeading_p)
        bHeading_m = [bHeading_p[0]/img_x_ratio_pixles_per_meters, bHeading_p[1]/img_y_ratio_pixles_per_meters]
        ang_line = bound_angle_rad(math.atan2(bHeading_m[1], bHeading_m[0])) # the obtained angle is with respect to the body frame x-axis
        ang_line_degree = ang_line/math.pi * 180
        self.angle_line_body = [ang_line, ang_line_degree]
        self.car_to_img_center_meters = bC_m

        return True
    def get_angle_line_in_body_frame(self):
        '''
        returns the angle of the line in the body frame
        [radian, degrees]
        '''
        return self.angle_line_body
    def get_line_center_in_body_frame_meters(self):
        '''
        returns the position of the center of the line in the body frame in meters
        [x_value, y_value]
        '''
        return self.car_to_img_center_meters
    def get_angle_line_in_img_frame(self):
        '''
        returns the angle of the line in the body frame
        [radian, degrees]
        '''
        return self.angle
    def get_line_center_img_pixels(self):
        '''
        returns the position of the center of the line in the camera frame in pixels
        [x_value, y_value]
        '''
        return self.center
    def get_color_arrow_img(self):
        '''
        returns the image of the line with the orientation arrow on it. The image is a color image (only the arrow has color). Also returns the indicator of the image as 'color'
        '''
        if self.img_arrow_color != []:
            return self.img_arrow_color, 'color'
        else:
            return [], ''
    def get_gray_arrow_img(self):
        '''
        returns the image of the line with the orientation arrow on it. The image is a gray image. Also returns the indicator of the image as 'gray'
        '''
        if img_arrow_color != []:
            return self.img_arrow_gray, 'gray'
        else:
            return [], ''

class lineDataPublisher:
    '''
    Publishes data about the line in line_img. In particular, it publishes the lateral and vertical position of the center of the line and the orientation of the line relative to the car's body frameself.
    '''
    def __init__(self):
        self.line_orientation = [] # orientation of the line relative to the car body frame. The first value is the angle in radian while the second is the angle in degrees
        self.line_center_position = [] # position of the line's center relative to the car body frame. Contains an x-y pair indicating the distance in meters
        self.line_data_pub = rospy.Publisher("/line_data", LineData, queue_size = 1)
    def import_data(self, ang, pos):
        '''
        populates the line orientation and line center position variables in self.
        '''
        if ((ang == []) | (pos == [])): # no data inputted
            rospy.logwarn("No data input")
            #print("No data input")
            self.line_orientation = []
            self.line_center_position = []
            return
        if ((len(ang) != 2) | (len(pos) != 2)):
            rospy.logwarn("Incorrect data input")
            #print("Incorrect data input")
            self.line_orientation = []
            self.line_center_position = []
            return

        self.line_orientation = ang
        self.line_center_position = pos
        return
    def publish_data(self):
        '''
        publishes line data to /line_data topic
        '''
        ang = self.line_orientation [:]
        pos = self.line_center_position [:]

        if ((self.line_orientation == []) | (self.line_center_position == [])):
            rospy.logwarn("No data to publish")
            #print("No data to publish")
            return

        self.line_data_pub.publish(LineData(ang[0], ang[1], pos[0], pos[1]))
        return

class imgPublisher:
    '''
    Publishes the image for the line containing the orientation of the line
    '''
    def __init__(self, topic_name):
        self.bridge = CvBridge()

        self.img = [] # image containing the line and its orientation
        self.img_indicator = '' # indicator for image ('gray', 'color', '')
        self.img_pub = rospy.Publisher(topic_name, Image, queue_size = 1)

    def import_img(self, img, indicator):
        '''
        imports an image and sets the indicator
        returns nothing
        '''
        if indicator == '':
            #print('Img does not have a color mode. Img will be set to []')
            rospy.logwarn("Img does not have a color mode")
            self.img = []
            self.img_indicator = ''
        else:
            #print('Importing image')
            self.img = img
            self.img_indicator = indicator
        return

    def publish_img(self):
        '''
        publishes the content of self.img to /cam/orientation_img if the image is not empty
        '''
        if self.img_indicator == '':
            ropy.logwarn('No image to publish')
            #print('No image to publish')
        elif self.img_indicator == 'gray':
            try:
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, 'mono8'))
                #print('Line image published')
            except CvBrdigeError as e:
                rospy.logerr(e)
                print(e)
        elif self.img_indicator == 'color':
            try:
                self.img_pub.publish(self.bridge.cv2_to_imgmsg(self.img, 'bgr8'))
                #print('Line image published')
            except CvBrdigeError as e:
                rospy.logerr(e)
                print(e)
        else:
            rospy.logerr('Something is wrong with the line image indicator')
            #print('Something is wrong with line image indicator')

        return


def main():
    rospy.init_node("process_line", anonymous=True)
    rate = rospy.Rate(30)

    # load parameters from ros parameter server
    nodename = "/process_line"
    line_topic_name = rospy.get_param(nodename + "/LineSub" + "/topic_name")
    threshold = rospy.get_param(nodename + "/LineParams" + "/threshold")
    width_line = rospy.get_param(nodename + "/LineParams" + "/width_of_line")
    width_gain = rospy.get_param(nodename + "/LineParams" + "/width_gain")
    arrow_scale = rospy.get_param(nodename + "/PlottingParams" + "/arrow_scale")
    square_size = rospy.get_param(nodename + "/CheckerboardParams" + "/square_size")
    car_body_frame_to_board = rospy.get_param(nodename + "/CheckerboardParams" + "/car_body_frame_to_board")
    path = rospy.get_param(nodename + "/path_to_corners")

    # load parameters from img_preprocessing node
    num_horizontal_checkerboard_corners = rospy.get_param("/img_preprocessing" + "/CheckerboardParams" + "/num_internal_checkerboard_corners_horizontally")
    num_vertical_checkerboard_corners = rospy.get_param("/img_preprocessing" + "/CheckerboardParams" + "/num_internal_checkerboard_corners_vertically")
    line_img_indicator = rospy.get_param('/img_preprocessing' + "/imgMode")


    # create objects
    line_fetcher = imgFetcher(line_topic_name, line_img_indicator)
    line_processor = lineProcessor()
    line_data_publisher = lineDataPublisher()
    arrowed_image_topic_name = "/cam/orientation_img"
    arrowed_img_publisher = imgPublisher(arrowed_image_topic_name)


    # update paramters for the line processor
    line_processor.set_param(threshold, width_line, width_gain, arrow_scale)
    while line_processor.param_update == False:
        line_processor.set_param(threshold, width_line, width_gain, arrow_scale)
        rospy.logfatal('Parameters were not set correctly')
        #return
    set_checkerboard_data_check = line_processor.set_checkerboard_data(num_horizontal_checkerboard_corners, num_vertical_checkerboard_corners, square_size, car_body_frame_to_board)
    while set_checkerboard_data_check == False:
        set_checkerboard_data_check = line_processor.set_checkerboard_data(num_horizontal_checkerboard_corners, num_vertical_checkerboard_corners, square_size, car_body_frame_to_board)
        rospy.logfatal('Checkerboard data incorrect. Shutting down')
        #return
    while line_processor.corners_from_file == []: # get the outer corners as found by the image_preprocessing node
        rospy.logwarn('Trying to find the corners of the checkerboard')
        line_processor.load_cornersTXT_data(path)
    try:
        while not rospy.is_shutdown():
            #####Pseudo code for section#####
            # get an image of the line
            # send it to the line processor
            # process the line to find center and orientation
            # map data to car body frame
            # publish line center and orientation
            #####END#####

            # get an image of the line
            line_img = line_fetcher.get_img()
            line_img_indicator = line_fetcher.get_img_indicator()
            # send it to the line processor
            import_check = line_processor.import_img(line_img, line_img_indicator)
            if import_check: # image was received by the line processor
                # process the line to find center and orientation
                data_obtained_check = line_processor.process_line_img()
                if data_obtained_check: # data was obtained
                    # map data to car body frame
                    transfor_line_data_to_body_check = line_processor.transfor_line_data_to_body() #map the data to the car's body frame
                    if transfor_line_data_to_body_check: # line center and heading are obtained
                        ang = line_processor.get_angle_line_in_body_frame()
                        pos = line_processor.get_line_center_in_body_frame_meters()
                        #publish line center and orientation
                        line_data_publisher.import_data(ang, pos)
                        line_data_publisher.publish_data()
                        img, indicator = line_processor.get_color_arrow_img()
                        arrowed_img_publisher.import_img(img, indicator)
                        arrowed_img_publisher.publish_img()

                else:
                    rospy.logerr('Line data not found')
            else:
                rospy.logerr('Image not imported')

            rate.sleep()

    except KeyboardInterrupt:
        rospy.logfatal("Keyboard Interrupt. Shutting down process_line node")
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down process_line node")
        pass

#
