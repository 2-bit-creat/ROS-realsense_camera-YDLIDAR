# ROS-realsense_camera/ YDLIDAR

Install ROS Package & Build Package
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/agilexrobotics/scout_ros.git
Install Dependencies
$ sudo apt install ros-melodic-teleop-twist-keyboard
$ sudo apt install ros-melodic-joint-state-publisher-gui
$ sudo apt install ros-melodic-ros-controllers
$ cd ~/catkin_ws
$ catkin_make

Setup CAN-To-USB
$ sudo modprobe gs_usb
$ sudo ip link set can0 up type can bitrate 500000
$ ifconfig -a (설정 확인을 위한 부분)
$ sudo apt install can-utils (최초 실행에만 필요)
$ candump can0 (데이터 입출력 확인을 위해 사용)
$ rosrun scout_bringup setup_can2usb.bash (위의 내용을 한 번에 실행)
$ rosrun scout_bringup bringup_can2usb.bash (재부팅 or USB 재연결시 실행)

Start Node
$ roslaunch scout_bringup scout_minimal.launch (using CAN)
