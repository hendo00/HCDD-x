1. cd kuacel_ws/src
2. git clone https://github.com/odriverobotics/ros_odrive.git
3. Clone this repo.
4. cd ..
5. colcon build --symlink-install
6. source ~/.bashrc
7. Power on the Odrive. #( Power necessary for CAN).
8. ros2 launch odrive_can odrive_can.launch.py #Init. the CAN e.g. can0).
9. Connect Joystick.
10. ros2 launch odrive_test odrv.launch.py  
