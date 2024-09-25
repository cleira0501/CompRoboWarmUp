import rclpy
from rclpy.node import Node
from warmup_project.teleop import TeleopNode
from obstacle_avoidance import ObstacleAvoidanceNode
from geometry_msgs.msg import Twist 

class FiniteStateControllerNode(Node):

    def __init__(self):
        super().__init__("finite_state_controller")
        self.teleop_node = TeleopNode()
        self.obstacle_node = ObstacleAvoidanceNode()

        self.create_timer(0.1, self.run_loop)

    def run_loop(self):
        key = self.teleop_node.get_key()
        if key == '\x03':  # Ctrl+C to exit
            self.teleop_node.drive(Twist())  # Stop
            return

        vel = self.teleop_node.direction(key)

        if not self.obstacle_node.avoid_obstacles().linear.x:  # if linear velocity = 0, obstacle is detected
            vel = self.obstacle_node.avoid_obstacles()  # Update with obstacle avoidance logic

        self.teleop_node.drive(vel)# else use teleop node as default mode

def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
