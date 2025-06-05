import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import time

from motordriver_msgs.msg import MotordriverMessage

from std_msgs.msg import Float32

class BatteryRedirector(Node):
    def __init__(self):
        super().__init__('battery_redirector')
        self.declare_parameter('battery_check_interval', 10.0) # Parametri jännitteen luvun aikavälille, sekuntia.
        self.publish_interval = self.get_parameter('battery_check_interval').value

        self.sub = self.create_subscription(MotordriverMessage, '/motor_data', self.callback, 10)
        self.pub = self.create_publisher(Float32, '/battery_voltage', 10)
        self.previous_time = time.time()

    def callback(self, msg):
        if time.time() - previous_time > self.publish_interval:
            battery_msg = Float32()
            battery_msg.data = msg.battery
            self.pub.publish(battery_msg)

def main(args=None):
  rclpy.init(args=args)
  batteryredirector_node = BatteryRedirector()
  try:
    rclpy.spin(batteryredirector_node)
  except KeyboardInterrupt:
    pass
  finally:
    batteryredirector_node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()

if __name__ == '__main__':
  main()
