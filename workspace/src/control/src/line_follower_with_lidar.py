#!/usr/bin/env python
import roslib
import sys
import rospy
from image_processing.msg import LineData
import time
from numpy import pi
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped as ADS
from sensor_msgs.msg import LaserScan #import lidar message

# TO BE MOVED TO A CONFIG FILE
heading_ctrl_Kp= 5.0
heading_ctrl_ctrl_max= 90.0
heading_ctrl_ctrl_min= -90.0

lateral_ctrl_Kp= 135.# was 150
lateral_ctrl_Ki= 0.#was 10
lateral_ctrl_Kd= 10.#was 10

lateral_ctrl_i_max= 10.0
lateral_ctrl_i_min= -10.0
lateral_ctrl_ctrl_max= 90.0
lateral_ctrl_ctrl_min= -90.0

wheel_radius = 0.0508 # in meters
# END OF THE CONFIG VARIABLES

col_ang = 23. # angle with respect to heading to scan for potential collisions (scan will be for [-col_ang, +col_ang]
col_dist = 0.8 # distance ahead to scan for potential collisions


class PID_ctrl:
    '''
    Generic discrete time PID controller with integrator anti-windup and control output saturation
    '''

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


class LineDataFetcher:
    '''
    Fetches data published to /line_data 
    '''
    def __init__(self):
        self.data_sub = rospy.Subscriber("/line_data", LineData, self.data_callback, queue_size = 1)
        self.line_data = []

    def data_callback(self, linedata):
        self.line_data = linedata
        return

    def get_linedata(self):
        return self.line_data
        
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


        
class LineFollower:
    '''
    Finds the steering angle for the car to follow a line based on /line_data (line heading and offset of car from line)
    '''
    def __init__(self):
        self.motor = 0.85
        self.heading_ctrl = []
        self.servo = []
        self.angle_radian = [] # that should be executed
        self.angle_degree = [] # that should be executed
        self.line_pos_x = [] # that should be executed
        #self.line_pos_y = [] # that should be executed
        self.cmd_dist = [] # when to execute a command relative to the car odometery
        self.cmd_req = [] #  required data to calculate commands (indexes coupled with cmd_dist)

    def import_linedata(self, line_data): 
        print(line_data)
        if (line_data == []):
            return
        self.angle_radian = line_data.angle_radian
        self.angle_degree = line_data.angle_degree
        self.line_pos_x = line_data.line_pos_x
        self.line_pos_y = line_data.line_pos_y
        print(self.angle_degree, self.line_pos_x)
        
    def calculate_ctrl(self, heading_ctrl, lateral_ctrl, timestamp):
        if self.angle_degree == []:
            return
        if self.line_pos_x == []:
            return
        #print('Calculating ctrl')
        heading_ctrl = heading_ctrl.apply_pid(self.angle_degree, 90.0 , timestamp) # desired is angle of line, and current is always 90
        lateral_ctrl = lateral_ctrl.apply_pid(0.08, self.line_pos_x, timestamp)
        ctrl = 0.0*heading_ctrl + 1.0*lateral_ctrl
        ctrl = -ctrl # add offset to remap ctrl from [-90, 90] to [0, 180]
        print('heading_ctrl', heading_ctrl)
        print('lateral_ctrl', lateral_ctrl)
        self.heading_ctrl = ctrl
    
    def set_servo_control(self):
        if self.heading_ctrl == []:
            self.servo = 30.0
        else:
            self.servo = self.heading_ctrl
        
    def get_ECU_data(self):
        return self.motor, self.servo

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

#class StrPub:
#    def __init__(self):
#        self.str_cmd = 0.0
#        self.str_pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", Float64, queue_size = 1)
#    
#    def set_str(self, str_cmd):
#        self.str_cmd = str_cmd
#        
#    def publish_str(self):
#        self.str_pub.publish(self.str_cmd)
        

def main():
    rospy.init_node("line_follower") #initialize ros node
    print("starting line follower")
    rate = rospy.Rate(30)
    

    # Instantiating classes 
    lidar_fetcher = LidarFetcher() # gets lidar data
    fetcher = LineDataFetcher() # fetches line data 
    publisher = VESCPublisher() # publishes control command to VESC
    line_follower = LineFollower() # passed line data thru PID controller to generate the control signal
    heading_ctrl = PID_ctrl(Kp = heading_ctrl_Kp, Ki = 0., Kd = 0., \
                            i_max = 0., i_min = 0., \
                            ctrl_max = heading_ctrl_ctrl_max, ctrl_min = heading_ctrl_ctrl_min) # control based on line angle 
    lateral_ctrl = PID_ctrl(Kp = lateral_ctrl_Kp, Ki = lateral_ctrl_Ki, Kd = lateral_ctrl_Kd, \
                            i_max = lateral_ctrl_i_max, i_min = lateral_ctrl_i_min, \
                            ctrl_max = lateral_ctrl_ctrl_max, ctrl_min = lateral_ctrl_ctrl_min) # control based on lateral offset 
    
    # Initializations
    timestamp = time.time()
    heading_ctrl.t_prev = timestamp
    heading_ctrl.t_curr = timestamp
    lateral_ctrl.t_prev = timestamp
    lateral_ctrl.t_curr = timestamp
    
    while not rospy.is_shutdown():
        timestamp = time.time()
        
        line_data = fetcher.get_linedata() # fetch line data
        lidardata = lidar_fetcher.get_lidardata() #fetch lidar data
        line_follower.import_linedata(line_data)#, trav_dist) # import line data to line follower
        line_follower.calculate_ctrl(heading_ctrl, lateral_ctrl, timestamp) # calculate the steering control
        line_follower.set_servo_control()
        motor, servo = line_follower.get_ECU_data() 
        
        servo_reg = servo /180 *pi
        servo_reg = min(max(servo_reg, -0.34), 0.34)
        motor_reg = min(max(motor, -2.0), 2.0) 
        
        print("Servo", servo)
        print("Servo_reg", servo_reg)
        print("Motor", motor)
        print("Motor_reg", motor_reg)
        print("t", timestamp)
        
        
        ads_msg = ADS()
        ads_msg.header.seq+=1
        ads_msg.header.stamp.secs = timestamp
        ads_msg.drive.steering_angle = servo_reg   
         
        if lidardata == []: # check if lidar data is populated
            ads_msg.drive.speed = motor_reg
        else:
            front_dists = lidardata.ranges[int(540 - col_ang*4):int(540 + col_ang*4 + 1)] # extract a range of front measurements 
            min_front = min(front_dists)
            if min_front < col_dist:
                ads_msg.drive.speed = 0.0
            else:
                ads_msg.drive.speed = motor_reg
        
        publisher.pub_vesc_msg(ads_msg)
    
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logfatal("ROS Interrupt. Shutting down line_follower node")
        pass
