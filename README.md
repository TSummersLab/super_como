# Super COMO
Super COMO is an autonomous car platfrom at the University of Texas at Dallas. It is a hardware upgrade over the [COMO platform](https://github.com/TSummersLab/como).

## Installation Guide

Before installing the Super COMO packages, perform a full JetPack installation on the Jetson TX2. For the package, JetPack 3.2.1 was installed. 

After installing JetPack, follow the steps below to setup Super COMO.
1. Configure repositories to allow "restricted," "universe," and "multiverse." Do this by following the instructions in [this link](https://help.ubuntu.com/community/Repositories/Ubuntu).
2. Clone the repository to your home directory. In a terminal window run:
```
cd ~ && git clone https://github.com/The-SS/super_como
```
3. In terminal, navigate to the `setup` folder in `super_como` by running:
``` 
cd ~/super_como/setup
```
4. (Optional) Configure git either manually or by running `git_config_setup.sh`. You can run the setup file as follows:
```
./git_config_setup.sh
```
5. Run the software installer as follows:
```
./software_installer.sh
```
Running the installer for the first time downloads and installs many packages, so it takes a while. Most of the script is automated and will not require user input until the very end when the ZED SDK is installed. For that installation, which is part of `software_installer.sh`, follow the instructions that appear in the terminal window.


By the end of the installation process, you will have the following packages and programs installed or updated:
* Nano
* Pluma
* VLC
* ROS Kinetic bare-bones 
* Certain ROS Kinetic packages for ARM architeture (the packages are listed in `super_como/setup/helper_scripts/install_ros_kinetic_packages.sh`
* Catkin_tools
* ZED SDK 2.3.3


