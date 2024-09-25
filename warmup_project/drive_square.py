import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
import time
import math

class DriveSquareNode(Node):

    def __init__(self):
        super().__init__("drive_square")
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
   
        # Define parameters
        self.total_turns = 4# 4 corners 
        self.linear_velocity = 0.2
        self.angular_velocity = 0.2
        self.side_length = 1
        self.turn_duration = (math.pi / 2) / self.angular_velocity# calculate the time need to turn 90 degrees
        self.straight_duration = self.side_length / self.linear_velocity# calculate the time needed to go 1m
        
        self.drive_square()

    def move_straight(self):
        vel = Twist()
        vel.linear.x = self.linear_velocity
        vel.angular.z = 0.0
        
      
        start_time = time.time()
        while time.time() - start_time < self.straight_duration:# when time hasn't reached the intended duration
            self.publisher.publish(vel)
            self.get_logger().info("Moving straight")
            time.sleep(0.1)  

    def turn(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = self.angular_velocity
        
      
        start_time = time.time()
        while time.time() - start_time < self.turn_duration:
            self.publisher.publish(vel)
            self.get_logger().info("Turning")
            time.sleep(0.1)  

    def drive_square(self):
        vel = Twist()
        for _ in range(self.total_turns):# the main loop to drive square
            self.move_straight()
            self.turn()
        vel.linear.x = 0.0
        vel.linear.z = 0.0

        
def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
