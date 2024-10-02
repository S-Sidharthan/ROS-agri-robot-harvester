# Agri-Bot for Greenhouse Automation
This project focuses on an agricultural robot (Agri-Bot) that navigates a simulated greenhouse environment to detect, pick, and place ripe tomatoes. The bot uses Intel RealSense D435i for object detection and depth estimation, with image thresholding for identifying ripe tomatoes. Additionally, Lidar is used for obstacle avoidance and navigation.

Table of Contents
Project Overview
Features
Installation
Usage
Images
Contributing
License
Project Overview
The Agri-Bot autonomously moves within a greenhouse simulation to identify ripe tomatoes based on color detection. It leverages the Intel RealSense D435i camera for depth sensing and object detection. The bot uses Lidar for obstacle avoidance, ensuring smooth navigation while avoiding collisions with objects in the greenhouse.

Features
Tomato Detection: Identifies ripe tomatoes using image thresholding based on red color.
Depth Estimation: Uses Intel RealSense D435i for accurate depth sensing and positioning.
Lidar-Based Navigation: Avoids obstacles during movement using Lidar data.
Autonomous Pick and Place: Capable of identifying ripe tomatoes and performing pick-and-place tasks.
Simulated Environment: A fully simulated greenhouse environment for testing.
Installation
Prerequisites
Ensure the following dependencies are installed:

ROS
realsense2_camera package
image_proc package for image processing
moveit package for motion planning
lidar package for navigation and obstacle avoidance
Steps
Clone the repository:

git clone https://github.com/yourusername/agri-bot.git
Build the project:

cd agri-bot
catkin_make
Source the workspace:

source devel/setup.bash
Usage
Start Agri-Bot
To launch the Agri-Bot in the greenhouse environment:


roslaunch yourpackage agri_bot.launch
Pick and Place Task
The Agri-Bot autonomously detects and picks ripe tomatoes:


rosrun yourpackage pick_place_task
Visualization
To visualize the robot and task progress in RViz:


roslaunch yourpackage rviz_visualization.launch
Images
Figure 1: Agri-Bot detecting ripe tomatoes.

Figure 2: Lidar-based navigation in the greenhouse.

Contributing
Contributions are welcome! Open issues or submit pull requests to improve the project.
