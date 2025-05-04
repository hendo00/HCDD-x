# HCDD-x
Control multiple odrive pros over CAN.
## Setup Instructions

```bash
cd kuacel_ws/src
git clone https://github.com/odriverobotics/ros_odrive.git
git clone https://github.com/hendo00/HCDD-x.git
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

## Add more odrives
1. Add the corresponding control node in odrv.launch.py (x:  the node id)
   ```bash
        Node(
            package='odrive_test',
            executable='odrive_control',
            name='odrive_controlx',
            parameters=[{'node_id': x}],
        ),
   ```
2. Add the corresponding can node in odrive_can.launch.py (x:  the node id)
   ```bash
    odrivex_can_node = Node(
	    package='odrive_can',
	    namespace='odrive_axisx',
	    executable='odrive_can_node',
	    name='can_node',
	    parameters=[{'node_id': x, 'interface': 'can0',}],
	)
   ```

