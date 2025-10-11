# tugbot_my  
ROS2 Gazebo for Tugbot.  

#### 開発環境  
PC: Ubuntu Mate 24.04  
ROS2 Jazzy  
Gazebo Hramonic  

#### Build.  
$ cd ~/colcon_ws-jazzy/src  
$ git clone https://github.com/tosa-no-onchan/tugbot_my.git  
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select tugbot_gazebo_my  
$ . install/setup.bash  

#### Run.  
$ sudo ufw disable  
#term1  
$ ros2 launch tugbot_gazebo_my warehouse.launch.py  
#term2  
$ rviz2  
add tf and etc.  
  
#term3  
control  
1)  Teleop keyboard  
$ ros2 run turtlebot3_teleop teleop_keyboard

#### 問題.  
まだ、3D LiDAR の PointClouds2 が、 Rviz2 上で目視確認できない。  
topic は出ているみたいだが、?  
--> 対応しました。  
