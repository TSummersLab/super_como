# Super COMO
Super COMO is an autonomous car platfrom at the University of Texas at Dallas. It is a hardware upgrade over the [COMO platform](https://github.com/TSummersLab/como).

## Installation Guide

Before installing the Super COMO packages, perform a full JetPack installation on the Jetson TX2. For the package, JetPack 3.2.1 was installed. 

After installing JetPack, follow the steps below to setup Super COMO.

1. Configure repositories:
Configure repositories to allow "restricted," "universe," and "multiverse." Do this by following the instructions in [this link](https://help.ubuntu.com/community/Repositories/Ubuntu).

2. Clone super_como :
Clone the repository to your home directory. In a terminal window run:
```
cd ~ && git clone https://github.com/The-SS/super_como
```

3. Navigate to setup folder:
In terminal, navigate to the `setup` folder in `super_como` by running:
``` 
cd ~/super_como/setup
```

4. Configure git (if not configured):
Configure git either manually or by running `git_config_setup.sh`. You can run the setup file as follows:
```
./git_config_setup.sh
```

5. Run the following scripts:
```
sudo ./install_programs.sh

sudo ./install_ros.sh

sudo ./install_zed_sdk.sh
```

6. Setup the device rules:
Run the rules set up script:
```
sudo ./rules_setup.sh
```

This script will reload the rules, but we recommend restarting the machine to ensure that the rules are in effect.

7. Run the build script:
The `build.sh` script at the root of the super_como package builds the catkin workspace. In a terminal window, navigate to the root of the super_como package then run the following command:
```
cd ~/super_como
./build.sh
```

8. Source the workspace:
Every time a new terminal window is open you have to source your workspace.
```
source ~/super_como/workspace/devel/setup.bash
```
Alternatively, you can add this to the bashrc file to avoid having to do that every time.
```
echo "source ~/super_como/workspace/devel/setup.bash" >> ~/.bashrc
```
Once this is performed you have to restart your terminal or source the bashrc file as follows:
```
source ~/.bashrc
```



