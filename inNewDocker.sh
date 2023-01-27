sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install ros-noetic-rosbridge-server -y
sudo apt-get install gedit -y
sudo apt-get install ros-noetic-video-stream-opencv -y
sudo apt-get install ros-noetic-rosserial -y
sudo apt-get install ros-noetic-moveit -y
sudo apt-get install ros-noetic-rviz-visual-tools -y
sudo apt-get install ros-noetic-moveit-visual-tools -y
cd /root/Desktop/ROS-Unity-OpenCR1.0
sudo mv master/ moveit_tutorials/
sudo mv noetic-devel/ panda_moveit_config/
cd 
mkdir -p /root/Desktop/catkin_ws/src
catkin_make
sudo ln -s /root/Desktop/ROS-Unity-OpenCR1.0 /root/Desktop/catkin_ws/src/ROS-Unity-OpenCR1.0
sudo ln -s /root/Desktop/catkin_ws/ /root/catkin_ws 
cd /root/Desktop/catkin_ws/
catkin_make
catkin_make install

