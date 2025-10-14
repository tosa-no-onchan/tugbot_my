# tugbot_my  
ROS2 Gazebo for Tugbot.  

#### 開発環境  
PC: Ubuntu Mate 24.04  
ROS2 Jazzy  
Gazebo Hramonic  

#### Build.  
$ cd ~/colcon_ws-jazzy/src  
$ git clone https://github.com/tosa-no-onchan/tugbot_my.git  
$ cd ..  
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

#### カスタマイズ.  

  tf-odom -> tf-basefoot_print の publish を止めるには、  
  tugbot_gazebo_my/models/tugbot.sdf を修正する。  

```
    <plugin name='ignition::gazebo::systems::DiffDrive' filename='ignition-gazebo-diff-drive-system'>
      <!-- wheels -->
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>

      <!-- kinematics -->
      <wheel_separation>0.5605</wheel_separation>
      <wheel_radius>0.195</wheel_radius>

      <!-- add by nishi -->
      <topic>cmd_vel</topic>

      <odom_topic>odom</odom_topic>
      <frame_id>odom</frame_id>
      <!-- <tf_topic>/tf</tf_topic> -->
      <!-- lego_loam_sr の時、 /tf が、バッティングするので、 /tf を /tf_gazebo にする。 by nishi 2025.10.13 -->
      <tf_topic>/tf_gazebo</tf_topic>
      <!-- lego_loam_sr の時、 下記、base_footprint を link しない。 by nishi 2025.10.13 -->
      <!-- <child_frame_id>base_footprint</child_frame_id> -->
      <odom_publisher_frequency>30</odom_publisher_frequency>
    </plugin>

```
