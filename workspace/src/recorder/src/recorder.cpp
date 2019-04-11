#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <hardware_interface/encoders.h>
#include <hardware_interface/ultrasonics.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/mat.hpp>
//#include <>


using namespace cv;

sensor_msgs::Imu imu_msg;
hardware_interface::ultrasonics us_msg;
sensor_msgs::LaserScan scan_msg;
sensor_msgs::Image msgr;
sensor_msgs::Image msgl;
Mat save_img;
 cv_bridge::CvImagePtr cv_ptr;


   /**
    * Call back functio for Imu. Argument is a pointer to the recieved message 
    */
  void imuCallback(const sensor_msgs::Imu &msg) {
    imu_msg = msg;
  }
  void usCallback(const hardware_interface::ultrasonics &msg){
    us_msg = msg;
  }
  

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    scan_msg = *scan; 
  }
  

  void imageRightRectifiedCallback(const sensor_msgs::Image &msgright) {
        msgr = msgright;
        
  }

  void imageLeftRectifiedCallback(const sensor_msgs::Image &msgleft) {
        msgl = msgleft;
  }


int main(int argc, char** argv){

    
  
    ros::init(argc, argv, "recorder"); 
    ros::NodeHandle n; 
    ROS_INFO("working");
    
    rosbag::Bag bag; 
    bag.open("data.bag", rosbag::bagmode::Write); 
    ros::Rate rate(30.0) ;
    
    
    ros::Subscriber imu_sub = n.subscribe("imu", 1, imuCallback); // subscribes to the imu and start the callback function
    ros::Subscriber sub_scanner = n.subscribe<sensor_msgs::LaserScan>("scan", 1, scanCallback); // subscribes to the lidar and start the callback function
    ros::Subscriber subRightRaw = n.subscribe("/zed/right/image_raw_color", 10, imageRightRectifiedCallback); 
    ros::Subscriber subLeftRaw = n.subscribe("/zed/left/image_raw_color", 10, imageLeftRectifiedCallback); 
    ros::Subscriber us_sub = n.subscribe("ultrasonics", 1, usCallback);
    
    while(ros::ok()){
       bag.write("imu", ros::Time::now(), imu_msg); // sending msg into the bag
       bag.write("scan", ros::Time::now(), scan_msg);
       bag.write("ultrasonics", ros::Time::now(), us_msg);
       bag.write("zed/left/image_raw_color", ros::Time::now(), msgl);
       bag.write("zed/right/image_raw_color", ros::Time::now(), msgr);
       ROS_INFO("recording data...");
       

       
        ros::spinOnce();
        rate.sleep();     
        
    }
    //ros::spin();
    bag.close();
 
    return 0;
}












































