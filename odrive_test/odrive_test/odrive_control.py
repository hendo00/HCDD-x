import rclpy
import subprocess
import signal
import time
from rclpy.node import Node
from sensor_msgs.msg import Joy
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage, ControllerStatus

class OdriveControl(Node):
    def __init__(self):
        super().__init__('odrive_control')
        self.shutdown_flag = False  # Shutdown flag
        self.declare_parameter('node_id', 0)  # Default ODrive node id
        node_id = self.get_parameter('node_id').get_parameter_value().integer_value

        # Build topic names based on node id
        self.controller_status_topic = f'/odrive_axis{node_id}/controller_status'
        self.control_message_topic = f'/odrive_axis{node_id}/control_message'
        self.axis_state_service = f'/odrive_axis{node_id}/request_axis_state'
        # Sub Joy, ControllerStatus & Pub ControlMessage
        self.status_sub = self.create_subscription(ControllerStatus, self.controller_status_topic, self.status_cb, 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_cb, 10)
        self.control_pub = self.create_publisher(ControlMessage, self.control_message_topic, 10)

        # Call the AxisState service [8: Control_mode, 1: Idle]
        self.axis_state_srv = self.create_client(AxisState, self.axis_state_service)

        # Vars
        self.control_modes = [1, 3]  # Torque, Velocity, Position
        self.current_mode = 0
        self.input_position = 0.0
        self.input_torque = 0.0
        self.input_velocity = 0.0
        self.pos_est = 0.0
        self.pressed = {i: False for i in range(12)}
        self.deadzone = 0.005
        self.last_button_press_time = {i: 0.0 for i in range(12)}
        self.button_cooldown = 0.2  # 200ms debounce time

        # Signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        # Set CLOSED_LOOP_CONTROL mode
        self.set_axis_state(8)

    def set_axis_state(self, state):
        while not self.axis_state_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for AxisState service...")


        req = AxisState.Request()
        req.axis_requested_state = state
        future = self.axis_state_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Axis state set to {state}")
    
    def status_cb(self, msg):
        self.pos_est = msg.pos_estimate
        self.get_logger().info(f"Position Estimate: {self.pos_est}")

    def joy_cb(self, msg):
        joy_x = msg.axes[1]
        if abs(joy_x) < self.deadzone:
            joy_x = 0.0

        
        current_time = time.time()
        # Increase/Decrease input values
        if msg.buttons[1] and current_time - self.last_button_press_time[1] > self.button_cooldown:
            self.pressed[1] = not self.pressed[1]
            self.last_button_press_time[1] = current_time
            self.get_logger().info("Press L1: -0.01, R1: +0.01")
            
        if msg.buttons[6] and self.pressed[1] and current_time - self.last_button_press_time[6] > self.button_cooldown:
            self.input_position -= 0.005
            self.input_torque -= 0.005
            self.input_velocity -= 0.005
            self.last_button_press_time[6] = current_time            

        elif msg.buttons[7] and self.pressed[1] and current_time - self.last_button_press_time[7] > self.button_cooldown:
            self.input_position += 0.005
            self.input_torque += 0.005
            self.input_velocity += 0.005
            self.last_button_press_time[7] = current_time

        # Hold input values
        elif msg.buttons[0] and current_time - self.last_button_press_time[0] > self.button_cooldown:
            self.input_position = self.input_position  # Hold position
            self.input_torque = self.input_torque  # Hold torque
            self.input_velocity = self.input_velocity  # Hold velocity
            self.pressed[0] = not self.pressed[0]
            self.last_button_press_time[0] = current_time

        # Joy input values
        elif not self.pressed[0] and not self.pressed[1]:
            self.input_position = joy_x * 20 
            self.input_torque = min(2.0, max(-2.0, joy_x*2)) # Limit torque to -2.0, 2.0 but joy_x is between -1, 1
            self.input_velocity = joy_x * 20
            self.get_logger().info(f"Joystick Axis: {joy_x}")   
        # Switch control mode
        if msg.buttons[3] and not self.pressed[3]:
            self.current_mode = (self.current_mode+1) % len(self.control_modes)
            self.get_logger().info(f"Control mode switched to: {self.control_modes[self.current_mode]}")
            self.pressed[3] = True
        elif not msg.buttons[3]:
            self.pressed[3] = False

        
        control_msg = ControlMessage()
        control_msg.control_mode = self.control_modes[self.current_mode]
        control_msg.input_mode = 1

        if self.control_modes[self.current_mode] == 1:
            control_msg.input_torque = self.input_torque
            self.get_logger().info(f"Input Torque: {control_msg.input_torque}")
        elif self.control_modes[self.current_mode] == 3:
            control_msg.input_pos = self.pos_est
            self.get_logger().info(f"Input Position: {control_msg.input_pos}")
        elif self.control_modes[self.current_mode] == 2:
            control_msg.input_vel = self.input_velocity
            self.get_logger().info(f"Input Velocity: {control_msg.input_vel}")
            
        self.control_pub.publish(control_msg)


    def signal_handler(self, signum, frame):
        self.get_logger().info("Shutting down...")
        self.set_axis_state(1)
        self.shutdown_flag = True 



def main():
    rclpy.init()
    node = OdriveControl()
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while not node.shutdown_flag:
            executor.spin_once(timeout_sec=0.1) 
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()