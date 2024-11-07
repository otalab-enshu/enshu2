## Repository for 精密工学基礎演習 第２週目

### Installation
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/otalab-enshu/enshu2.git

cd ..
catkin_make
```

### Example
PC side:
```bash
roslaunch enshu2 detect_marker.launch
roslaunch enshu2 get_marker_distance.launch
rosrun enshu2 robot_driving
```
Turtlebot with RealSense D455 side:
```bash
roslaunch turtlebot_realsense.launch
```
