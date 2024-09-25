import rclpy
from rclpy.node import Node
from warmup_project.teleop import TeleopNode
from warmup_project.obstacle_avoidance import ObstacleAvoidanceNode
from geometry_msgs.msg import Twist 
import termios
import sys
from sensor_msgs.msg import LaserScan
class FiniteStateControllerNode(Node):

    def __init__(self):
        super().__init__("finite_state_controller")
        self.teleop_node = TeleopNode()
        self.obstacle_node = ObstacleAvoidanceNode()
        rclpy.spin(self.teleop_node)
        rclpy.spin(self.obstacle_node)
        self.create_subscription(LaserScan, 'scan',self.latest_scan, 10)
        self.create_timer(0.1, self.run_loop)
        self.scan = LaserScan()

    def latest_scan(self, msg):
        self.scan = msg
    

    def run_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        key = self.teleop_node.getKey(settings)
        if key == '\x03':  # Ctrl+C to exit
            self.teleop_node.drive(Twist())  # Stop
            return

        vel = self.teleop_node.direction(key)

        if self.obstacle_node.current_vel.linear.x == 0:  # if linear velocity = 0, obstacle is detected
            print("obstacle")# this does not work
            self.obstacle_node.process_scan(self.scan)  # Process the latest scan
            vel = self.obstacle_node.current_vel  # Update with obstacle avoidance logic

        self.teleop_node.drive(vel)# else use teleop node as default mode

def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
