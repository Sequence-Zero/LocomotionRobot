import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_pwm', 200)
        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_angular', 1.5)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.get_logger().info(f'Opened serial port {port} at {baud}')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.2, self.watchdog_callback)

    def listener_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        max_pwm = int(self.get_parameter('max_pwm').value)
        max_linear = float(self.get_parameter('max_linear').value)
        max_angular = float(self.get_parameter('max_angular').value)

        v = clamp(msg.linear.x, -max_linear, max_linear)
        w = clamp(msg.angular.z, -max_angular, max_angular)

        left_norm = (v / max_linear) - (w / max_angular)
        right_norm = (v / max_linear) + (w / max_angular)

        left_pwm = int(clamp(left_norm, -1.0, 1.0) * max_pwm)
        right_pwm = int(clamp(right_norm, -1.0, 1.0) * max_pwm)

        cmd = f"M {left_pwm} {right_pwm}\n"
        self.ser.write(cmd.encode("utf-8"))

    def watchdog_callback(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > 0.6:
            try:
                self.ser.write(b"M 0 0\n")
            except Exception:
                pass

def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.write(b"M 0 0\n")
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
