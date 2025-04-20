## Setup Instructions

```bash
cd kuacel_ws/src
git clone https://github.com/odriverobotics/ros_odrive.git
git clone https://github.com/hendo00/HCDD.git
cd ..
colcon build --symlink-install
source ~/.bashrc
```

## Running

1. Power on the ODrive (CAN required).
2. Launch ODrive CAN interface:

    ```bash
    ros2 launch odrive_can odrive_can.launch.py
    ```

3. Connect the joystick.
4. Launch the test node:

    ```bash
    ros2 launch odrive_test odrv.launch.py
    ```
