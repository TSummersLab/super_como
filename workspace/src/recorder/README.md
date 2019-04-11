### Recording the Basic Data 

This program record the basic out put of the main components including, Zed left and right images, LIDAR, IMU, and Teensy. It was created to reduce the data recorded as recording every message creates a huge file. 
#### Running Recorder

To run the program us rosrun to run the .cpp file in a terminal.
```
rosrun recorder recorder
```
This command will launch the file and start recording.

#### Recorded Topics

THis code record the following topics
 * scan                          : Message holding the raw LIDAR data
 * /zed/right/image_raw_color    : Message holding right ZED image
 * /zed/left/image_raw_color     : Message holding left ZED image
 * ultrasonics                   : Message holding the data from the ultrasonics
 * imu                           : Message holding the IMU data

#### Recording other Topics

You can record other topics using ros bag files to do so run the following command. 
```
rosbag record topic_name -O name.bag
```
This will create a bag named name and save topic_name to it. You can add a path to the bagfile name. /home/data/name.bag. you can also add more topics to be recorded.
```
rosbag record topic1 topic2 topic3 -O /home/data/name.bag
```
