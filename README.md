# JCAR-Competition
Kinova robot grasp package for JCAR Competition

# Introduction
This Project is done by freeman team, the repository includes two parts: 1.A simulation for kinova grasp 2. Real kinova grasp.

# Author
3D model files are built by Jianbo Zhang & LiuLong Ma. The files can be find [here](https://github.com/marooncn/robot_grasp#introduction)

Kinova grasp simulation is built by KaiXiang Xu

# Environment
This Package is tested on ubuntu14.04, ros indigo

# Pre-install
## python-pcl
download python-pcl from [here](https://github.com/strawlab/python-pcl)

sudo pip install cython

python setup.py build

sudo python setup.py install

sudo apt-get install pcl-tools

## Kinova-ros
You can find kinova-ros package [here](https://github.com/Kinovarobotics/kinova-ros)

# Run demo by yourself
## Compile the package
1. Go to the package named robot_grasp, find the CmakeLists file inside. Comment out all the add_execytable and target_link

2. catkin_make the workingspace

3. recover the items that are commented out before

4. catkin_make the workingspace angin

## Run
roalaunch robot_grasp START_Gazebo.launch







