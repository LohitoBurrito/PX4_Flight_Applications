# PX4 Flight Applications

## Abstract
This repository contains my implementation of various uav applications where the uav will perform various tasks given a certain control code. 

Next Tasks:
1. Create a LiDAR application
2. Learn and Create SLAM based application
3. Learn Behavior Trees
4. Dockerize as much of this project as possible
5. Create an application where I have a seperate GUI built on C++ or Rust where I can provide commands for the Drone on Gazebo
6. Figure out how to migrate from Gazebo Classic to Gazebo Ignition so I can work on Multi-UAV simulation (or learn ArduPilot)

Note that my main ROS2 C++ control code is located within
`
./uav_app/src/px4_ros_com/src/app
`

## ⭐ Base Repositories & Documentation ⭐ <br />
① [PX4 Documentation](https://docs.px4.io/main/en/ros2/user_guide.html) <br />
② [PX4 ROS2 Message Repo](https://github.com/PX4/px4_msgs) <br />
③ [PX4 ROS2 Communication Repo](https://github.com/PX4/px4_ros_com) <br />

## Test Videos
- [Recon Test 1](https://www.youtube.com/watch?v=n0gshWVHZww)
- [Fire Test 1](https://www.youtube.com/watch?v=nykbBwVcKCo)

## Installation
For more information on installation, go to the PX4 Documentation link in credits.
### WSL Installation
Open windows powershell and run:
```
wsl --install -d Ubuntu-22.04
```
### PX4 Development Environment Installation
To retrieve the PX4 ROS2 World and Code, run the following in a new ubuntu terminal
```
sudo apt install cmake
sudo apt install make
sudo apt install python3-pip

cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```
### ROS2 Humble Installation
To install ROS2 Humble run:
```
cd
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```
### Micro XRCE-DDS Agent Installation
In a new Ubuntu 22.04.5 LTS terminal, run:
```
cd
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
### Q Ground Control (QGC) Installation
Run the following in a new Ubuntu 22.04.5 LTS terminal:
```
cd
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
Download the following [link](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage) to retrieve the Q Ground Control App Image. Then run the following to turn the app image into a linux executable:
```
cd
chmod +x ./QGroundControl.AppImage
```
### PX4 ROS2 Workspace Installation
Ensure you installed the PX4 Development Environment before performing this step. In a new terminal, run the following:
```
cd
git clone https://github.com/LohitoBurrito/PX4_Flight_Applications.git
mv PX4_Flight_Applications/* . && rm -rf PX4_Flight_Applications
cd uav_app
chmod +x setup.sh
cd
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-bridge
cp -r ./gazebo_package/red_car ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/red_car
cp -r ./gazebo_package/building ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/building
cp -r ./gazebo_package/omni_light ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/omni_light
cp -r ./gazebo_package/hydrant ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/hydrant
cp -r ./gazebo_package/search_and_rescue.world ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/search_and_rescue.world
cp -r ./gazebo_package/fire.world ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/fire.world
cp -r ./gazebo_package/sitl_targets_gazebo-classic.cmake ./PX4-Autopilot/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake
sudo rm -rf gazebo_package
```
## Run Instructions
If you just performed installation, you can close all 4 terminals, and run 4 new Ubuntu 22.04.5 LTS terminals. For the Terminal 3 and 4 commands, you need to fill {world} and {executable} with the appropriate parameters. Below shows a table of combinations of world and executables.

<div align="center">
  
| Executable | World |
|     :---:      |     :---:      |
|  fireforce  |  iris_depth_camera__fire  |
|  reconnaissance  |  iris_downward_depth_camera__search_and_rescue |

</div>

### Terminal 1
To start the Micro XRCE-DDS Agent, run:
```
cd Micro-XRCE-DDS-Agent/
MicroXRCEAgent udp4 -p 8888
```
### Terminal 2
To start QGC App Image, run the following:
```
./QGroundControl.AppImage
```
### Terminal 3
Start the Gazebo world by running the following. Note that if it says "gzserver not ready yet, trying again!," keep waiting. (NOTE: If this is your first time running the world, it may take a bit due to creating the build folder)
```
cd PX4-Autopilot/
make px4_sitl gazebo-classic_{world}
```
### Terminal 4
Ensure you have the other terminals running before running this set of terminal commands. Run the simulation by executing the following commands (NOTE: If this is your first time running the simulation, it may take a bit due to creating the build, install, and log folder)
```
cd uav_app/
./setup.sh {executable}
```

## Acknowledgments

This project uses **PX4** for flight control, which is licensed under the BSD-3-Clause License.  
For more information, visit [PX4's GitHub Repository](https://github.com/PX4/PX4-Autopilot).

This project uses Ultralytics' YOLOv5, which is licensed under the GNU General Public License v3.0 (GPL-3.0).
For more information, visit the [Ultralytics Yolov5 repository](https://github.com/ultralytics/yolov5)
