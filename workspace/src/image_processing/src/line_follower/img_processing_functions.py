#!/usr/bin/env python
'''
Contains all the functions used by the image processing files

Author:
Sleiman Safaoui
Email:
snsafaoui@gmail.com
sxs169833@utdallas.edu
Git:
The-SS

Date:
May 30, 2018
'''
from __future__ import print_function
import cv2
import numpy as np
import math
import rospy # these functions are maked



def import_file(file_name):
    '''
    returns the content of a file with two data entries per row.

    input:
            string containing text file name either relative to the folder the function is called from or with the absolute path
    output:
            array containing data
    '''
    data = np.loadtxt(file_name, delimiter=',', usecols=range(2), comments='#')
    return data



def export_file(file_name, data):
    '''
    overwrites the content of a file

    input:
            file_name: str containing file name (relative to where the function is called or absolute)
            data: 2D numpy array containing data to be stored in the file
    output:
            nothing
    '''
    np.savetxt(file_name, data, delimiter=',')
    return



def l2dist(pt1, pt2):
    '''
    finds the Euclidean distance between two points with x and y coordinates

    input:
            both inputs are lists or arrays with two elements only
    output:
            a floating point number
    '''
    return float((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)**0.5



def get_checkerboard_corners(img, num_corners_cols,num_corners_rows):
    '''
    gets all internal checkerboard corners of a checkerboard

    input:
            img: 8-bit grayscale or color image
            num_corners_cols: integer number of horizontal checkerboard corners
            num_corners_rows: integer number of vertical checkerboard corners

    output:
            a list either empty or containing the internal corners of the checkerboard
    '''
    img_pts = []
    ret, corners = cv2.findChessboardCorners(img, (num_corners_cols,num_corners_rows), cv2.CALIB_CB_FILTER_QUADS + cv2.CALIB_CB_ADAPTIVE_THRESH)
    # print('ret',ret)
    if (ret == False): # if pattern not detected, repeat process with new image
        # get new image
        return([])
        # ret, corners = cv2.findChessboardCorners(img, (num_corners_cols,num_corners_rows),None)
    img_pts.append(corners)
    # Draw and display the corners
    img2 = cv2.drawChessboardCorners(img, (num_corners_cols,num_corners_rows), corners,ret)
    print('corners:   ',corners)
    cv2.imshow('img2',img2)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return img_pts



def select_checkerboard_points(corners, cols, rows):
    '''
    selects the outer four checkerboard corners:
    .-------->X
    |  3    2
    |  0    1
    V
    Y

    input:
            corners: list containing the internal checkerboard corners as returned by cv2.findChessboardCorners
            cols: integer number of horizontal checkerboard corners
            rows: integer number of vertical checkerboard corners

    output:
            a list containing the outermost internal corners of the checkerboard
    '''
    ordered_pts = np.zeros((4, 2), dtype = "float32")
    ordered_pts[3] = corners[0][0][0]
    ordered_pts[2] = corners[0][cols-1][0]
    ordered_pts[1] = corners[0][cols*rows-1][0]
    ordered_pts[0] = corners[0][cols*(rows-1)][0]
    #arrange_rect_pts(ordered_pts)
    return ordered_pts



def arrange_rect_pts(points):
    '''
    arranges four corners of a rectangle in the following order
    .-------->X
    |  3    2
    |  0    1
    Y

    input:
            a 4x2 list containing data for four points with x and y value

    output:
            a 4x2 list containing data for four points with x and y value
    '''

    ordered_pts = np.zeros((4, 2), dtype = "float32")
    r_x_vals = [points[0][0], points[1][0], points[2][0], points[3][0]]
    r_y_vals = [points[0][1], points[1][1], points[2][1], points[3][1]]

    sort_x_idx = np.argsort(r_x_vals)
    # points 1 and 2
    max_x_a_idx = sort_x_idx[3]
    max_x_b_idx = sort_x_idx[2]
    if (r_y_vals[max_x_a_idx] > r_y_vals[max_x_b_idx]): # a is 1 and b is 2
        ordered_pts[1] = [r_x_vals[max_x_a_idx], r_y_vals[max_x_a_idx]]
        ordered_pts[2] = [r_x_vals[max_x_b_idx], r_y_vals[max_x_b_idx]]
    else: # b is 1 and a is 2
        ordered_pts[1] = [r_x_vals[max_x_b_idx], r_y_vals[max_x_b_idx]]
        ordered_pts[2] = [r_x_vals[max_x_a_idx], r_y_vals[max_x_a_idx]]

    # points 0 and 3
    min_x_a_idx = sort_x_idx[1]
    min_x_b_idx = sort_x_idx[0]
    if (r_y_vals[min_x_a_idx] > r_y_vals[min_x_b_idx]): # a is 0 and b is 3
        ordered_pts[0] = [r_x_vals[min_x_a_idx], r_y_vals[min_x_a_idx]]
        ordered_pts[3] = [r_x_vals[min_x_b_idx], r_y_vals[min_x_b_idx]]
    else: # b is 3 and a is 0
        ordered_pts[0] = [r_x_vals[min_x_b_idx], r_y_vals[min_x_b_idx]]
        ordered_pts[3] = [r_x_vals[min_x_a_idx], r_y_vals[min_x_a_idx]]

    return (ordered_pts)



def find_intersection(slope1, slope2, x1, y1, x2, y2):
    '''
    finds the intersction of two lines given their slopes and a point belonging to them

    input:
            slope1: slope of line 1 of float type
            slope2: slope of line 2 of float type
            x1, y1: point belonging to line 1. Each is a float
            x2, y2: point belonging to line 2. Each is a float

            the two lines must have a single intersection
            equations of line are in the form:
                y - slope1 * x = y1 - slope1 * x1
    output:
            an array containing the point of intersection (xi, yi). Both elements are floats
    '''
    coef = np.array([[-slope1, 1], [-slope2, 1]])
    res = np.array([y1-slope1*x1, y2-slope2*x2])
    I = np.linalg.solve(coef, res)
    return I



def extend_corners(x_max, y_max, corners):
    '''
    Extends four corners of a tetragon so that the rightmost point has x=x_max and the top most point has y=y_max
    When applying to the outer four corners of the checkerboard to expand the region laterally, the shape must be close to a trapezoid

    input:
            x_max: integer number representing the maximum width
            y_max: integer number representing the maximum height
            corners: 4x2 list containing the x and y values of four corners. Data is float type

    output:
            a 4x2 list containing the x and y values of the extended four corners. Data is float type
    '''
    x0 = float(corners[0][0])
    y0 = float(corners[0][1])
    x1 = float(corners[1][0])
    y1 = float(corners[1][1])
    x2 = float(corners[2][0])
    y2 = float(corners[2][1])
    x3 = float(corners[3][0])
    y3 = float(corners[3][1])
    x_max = float(x_max)
    y_max = float(y_max)

    # intersection between current left and right edges
    slope_l = (y3-y0)/(x3-x0) # slope of current left line
    slope_r = (y2-y1)/(x2-x1) # slope of current right line
    xi, yi = find_intersection(slope_l, slope_r, x0, y0, x1, y1)

    # finding expanded points for 0 and 1 (0n and 1n)
    x0n = 0.0
    x1n = x_max-1
    slope_b = (y1-y0)/(x1-x0) # slope of bottom line
    y0n = y0 + slope_b * (x0n - x0)
    y1n = y0 + slope_b * (x1n - x0)

    # find expanded points for 2 and 3 (2n  and 3n) by finding intersection between top line and the line formed by joining the point of intersection with 0n and 1n
    slope_ln = (yi-y0n)/(xi-x0n) # slope of new left edge
    slope_rn = (yi-y1n)/(xi-x1n) # slope of new right edge
    slope_t = (y3-y2)/(x3-x2) # slope of top edge

    x2n, y2n = find_intersection(slope_rn, slope_t, x1n, y1n, x2, y2)
    x3n, y3n = find_intersection(slope_ln, slope_t, x0n, y0n, x3, y3)

    expanded_corners = np.zeros([4, 2])
    expanded_corners[0][0] = int(x0n)
    expanded_corners[0][1] = int(y0n)
    expanded_corners[1][0] = int(x1n)
    expanded_corners[1][1] = int(y1n)
    expanded_corners[2][0] = int(x2n)
    expanded_corners[2][1] = int(y2n)
    expanded_corners[3][0] = int(x3n)
    expanded_corners[3][1] = int(y3n)
    print('expanded_corners', expanded_corners)

    for i in range(4):
        check = 0
        check += (expanded_corners[i][0] >= x_max)
        check += (expanded_corners[i][0] < 0)
        check += (expanded_corners[i][1] >= y_max)
        check += (expanded_corners[i][1] < 0)
        if check > 0:
            print(i,"!")
            print(expanded_corners[i][0] >= x_max)
            print(expanded_corners[i][0] < 0)
            print(expanded_corners[i][1] >= y_max)
            print(expanded_corners[i][1] < 0)
            expanded_corners = []
            return expanded_corners

    return expanded_corners.astype(dtype="float32")



def bound_angle_rad(ang):
    '''
    bounds angle between 0 and pi
    Apply this to the result of atan2
        atan2: (x,y) |---> [-pi, pi]
        bound_angle_camera_rad: ang in [-pi, pi] |---> [0, pi]

    input:
            ang: float
    output:
            float
    '''
    if ang < 0:
        ang += math.pi
    return ang



def applyPtTransformation_2D(H, pt):
    '''
    applies a transformation to a point in 2D

    input:
            H: 4x4 array representing the transformation matrix
            pt: a list or array containing the x and y values of a point

    output:
            list containing the x and y value of the transformed point
    '''
    pt_homo = np.array([pt[0], pt[1], 0, 1])
    res_homo = np.dot(H, np.transpose(pt_homo))
    return [res_homo[0], res_homo[1]]



def applyVectorTransformation_2D(H, vector):
    '''
    applies a transformation to a vector in 2D

    input:
            H: 4x4 array representing the transformation matrix
            pt: a list or array containing the x and y values of a vector

    output:
            list containing the x and y value of the transformed vector
    '''
    vector_homo = np.array([vector[0], vector[1], 0, 0])
    res_homo = np.dot(H, np.transpose(vector_homo))
    return [res_homo[0], res_homo[1]]



def color_mode_to_encoding(mode):
    '''
    changes color mode to encoding value
    'color' --> 'bgr8'
    'gray' --> 'mono8'

    input:
            mode: str containing either 'color' or 'gray'

    output:
            string: (mono8 or bgr8) if conversion works or '' if it fails
    '''

    if mode == 'gray': # 8 bit grayscale image
        return 'mono8'

    if mode == 'color': # 8 bit color image
        return 'bgr8'

    return -1 # invalid input



def encoding_to_color_mode(encoding):
    '''
    changes color mode to encoding value
    'bgr8'  --> 'color'
    'mono8' --> 'gray'

    input:
            encoding: str containing either 'bgr8' or 'mono8'

    output:
            string: (gray or color) if conversion works or '' if it fails
    '''

    if encoding == 'mono8': # 8 bit grayscale image
        return 'gray'

    if encoding == 'bgr8': # 8 bit color image
        return 'color'

    return -1 # invalid input



# .
