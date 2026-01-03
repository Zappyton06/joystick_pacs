import rclpy
import pygame
from rclpy.node import Node
from nemo_interfaces.msg import RovCommands
import serial
import time
from std_msgs.msg import Bool

START_BIT = 0xAA
STOP_BIT = 0x55

class joystickReadWrapper:
    def __init__(self,controller_id):
        pygame.init()
        pygame.joystick.init()

        self.last_button_states = {}
        self.toggle_states = {}

        
        self.controller = pygame.joystick.Joystick(controller_id)
        if not self.controller.get_init():
            self.controller.init()

        self.pause_read = False

    def readAxis(self,axis_id):
        rawAxisval = None
        try:
            pygame.event.pump()
            rawAxisval = self.controller.get_axis(axis_id)
            return rawAxisval
        except self.pause_read:
            pass


class ThrusterSerialNode(Node):
    def __init__(self):
        super().__init__('thruster_serial_node')

        self.declare_parameter('SerialPort', '/dev/ttyUSB0')
        self.serial_port = self.get_parameter('SerialPort').get_parameter_value().string_value

        self.arming_state = False
        
        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=0.1)
            time.sleep(2)
            self.get_logger().info("✅ Serial port connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Serial open failed: {e}")
            self.ser = None

        self.subscription_cmd = self.create_subscription(
            RovCommands, '/nemo_auv/input_cmd', self.cmd_callback, 10
        )

        """self.subscription_joy = self.create_subscription(
            Bool,'/arm_status',self.joy_callback, 10
        )"""

        # Timer: checks joystick state every 50ms (non-blocking)

    def joy_callback(self,msg):

        msg = Bool()
        state = msg.data
        if state:
            self.arming_state = not self.arming_state 

        

    def cmd_callback(self, msg):
        """Send thruster data only."""
        if not self.ser or not self.ser.is_open:
            self.get_logger().warn("Serial not available.")
            return
        


        surge_val = int(msg.surge * 100) & 0xFF
        heave_val = int(msg.heave * 100) & 0xFF
        sway_val = int(msg.sway * 100) & 0xFF
        yaw_val = int(msg.yaw * 100) & 0xFF
        #armstatus = 0xFF if self.armStatus else 0x00

        packet = bytearray([0xAA, surge_val, sway_val, yaw_val, heave_val, 0x55])
        
        try:
            self.ser.write(packet)
            print(f"written packet {packet}")
        except Exception as e:
            self.get_logger().error(f"❌ Serial write failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()