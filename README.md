## Repository for 精密工学基礎演習 第３週目

### Installation
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/asamalab/enshu3.git

# Run the following as well if you don't have enshu_msgs
git clone https://github.com/asamalab/enshu_msgs.git

cd ..
catkin_make
```