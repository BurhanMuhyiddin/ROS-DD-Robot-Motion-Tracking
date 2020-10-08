# Differential Drive Robot Motion Tracking in ROS
This project demonstrates how to apply motion tracking, in this case a walking person, in differential-drive robot, in this case Pioneer p3dx, by simulating it in Coppeliasim(Vrep) through ROS.

# Getting Started
Here I will describe how to run the project

## Prerequisites
You will need to donwload and install Coppeliasim and ROS

# Running The Tests
Firstly, run the Coppeliasim and open the motionTrackingFinal.ttt scene from scenes inside the package. Then, after running the scene and after adding the package to your workspace, launch the start_project.launch. And then just watch :)

# Some Explanation
Motion detection has been done on OpenCv library in Python. The subsequent two frames have been subtracted in order to detect the motion. The offset between the center of the moving object and the camera has been calculated and PID applied for correct tracking. Additionally distance is calculated in order to keep robot some amount far from the person, so person can be seen in the camera and processing can be done.

# Author
Burhan Muhyiddin
