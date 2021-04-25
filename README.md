# r2auto_nav
ROS2 auto_nav code for EG2310



Preliminary 

1. First, install and set up Ubuntu 20.04 and ROS 2 Foxy on your laptop. 

2. Istall ROS 2 on the Raspberry Pi (R-Pi) on the TurtleBot.

3. Setup ssh for the rpi https://phoenixnap.com/kb/enable-ssh-raspberry-pi

4. Git clone the following repository into your laptop: https://github.com/leeyikai/r2auto_nav.git under this directory:
~/colcon_ws/build/auto_nav/





Main operation 

5. Once the turtlebot has successfully booted, ssh into the rpi.

6. Launch the command: “rosbu” on the rpi and wait until the print statement “run!” can be seen. 

7. Run the launcher publisher code on the rpi on a new terminal with the following command: 
ros2 run py_pubsub talker

8. Run the rslam on the laptop on a new terminal with the following command: 
ros2 launch turtlebot3_cartographer cartographer.launch.py

9. Run the navigation code on the laptop on a new terminal with the following command: 
ros2 run autonav eg2310finals

9. Save the map after finish navigation on the laptop on a new terminal with the following command: 
ros2 run nav2_map_server map_saver_cli -f ~/map

From https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#save-map (foxy)
