# MIE443-Turtlebot-Contests
To apply robotics principles and algorithms on the turtlebot4 and the arm101 to accomplish various contests given by the course MIE443 at UofT

# To launch
chmod +x launch_contest1.sh
./launch_contest1.sh

# Linux Commands to switch between IRL and Simulation
$ sudo nano /etc/turtlebot4/setup.bash #edit rwm_implementation
$ source ~/.bashrc #save changes
$ echo $RMW_IMPLEMENTATION #double check current rmw_implementation
$ echo $ROS_DOMAIN_ID #computer id should be 14
$ ros2 topic list
$ ros2 daemon stop
$ ros2 daemon start
$ ros2 topic list #show command list
$ ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}" #undocking turtlebot
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard #enable manual keyboard controls
