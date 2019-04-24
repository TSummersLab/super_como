#!/bin/bash

# adds rules file for the sensors. See documentation for details

echo -e "ATTRS{idProduct}==\"01a7\", ATTRS{idVendor}==\"1546\", ATTRS{product}==\"u-blox 7 - GPS/GNSS Receiver\", SYMLINK+=\"gps\" #GPS \nATTRS{idProduct}==\"9d0f\", ATTRS{idVendor}==\"1b4f\", ATTRS{manufacturer}==\"SparkFun\", ATTRS{product}==\"SFE 9DOF-D21\", SYMLINK+=\"imu\" #IMU \nATTRS{idProduct}==\"5740\", ATTRS{idVendor}==\"0483\", ATTRS{manufacturer}==\"STMicroelectronics\", ATTRS{product}==\"ChibiOS/RT Virtual COM Port\", SYMLINK+=\"vesc\" #VESC \nATTRS{idProduct}==\"0483\", ATTRS{idVendor}==\"16c0\", ATTRS{manufacturer}==\"Teensyduino\", ATTRS{product}==\"USB Serial\", SYMLINK+=\"teensy\" #Teensy \n \nKERNEL==\"video*\", KERNELS==\"1-2*\", ATTRS{idProduct}==\"9230\", ATTRS{idVendor}==\"05a3\", ATTRS{product}==\"USB 2.0 Camera\", SYMLINK+=\"elp/back\" #back ELP camera \nKERNEL==\"video*\", KERNELS==\"5-1\", ATTRS{idProduct}==\"9230\", ATTRS{idVendor}==\"05a3\", ATTRS{product}==\"USB 2.0 Camera\", SYMLINK+=\"elp/front\" #front ELP camera \nKERNEL==\"video*\", KERNELS==\"9-1\", ATTRS{idProduct}==\"9230\", ATTRS{idVendor}==\"05a3\", ATTRS{product}==\"USB 2.0 Camera\", SYMLINK+=\"elp/left\" #left ELP camera \nKERNEL==\"video*\", KERNELS==\"7-1\", ATTRS{idProduct}==\"9230\", ATTRS{idVendor}==\"05a3\", ATTRS{product}==\"USB 2.0 Camera\", SYMLINK+=\"elp/right\" #right ELP camera" | sudo tee /etc/udev/rules.d/99-device-rules.rules

sudo udevadm control --reload-rules && udevadm trigger


