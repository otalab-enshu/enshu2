## Repository for 精密工学基礎演習 第３週目

### Installation
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/asamalab/enshu3.git

# Send launch file to turtlebot3
scp enshu3/launch/*_realsense.launch ubuntu@{IP_TURTLEBOT3}:~

# Run the following as well if you don't have enshu_msgs
git clone https://github.com/asamalab/enshu_msgs.git

cd ..
catkin_make
```

### Example
PC side:
```bash
roslaunch enshu3 yolox_node.launch
rosrun enshu3 robot_driving
```
Turtlebot with RealSense D455 side:
```bash
roslaunch turtlebot_realsense.launch
```