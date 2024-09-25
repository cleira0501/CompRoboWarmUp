
import rclpy  # convenience python library for interacting with ROS2
from rclpy.node import Node  # generic Node class for interacting with ROS2
from neato2_interfaces.msg import Bump  # local package call for a Bump type message format
from geometry_msgs.msg import Twist  # ROS package call for a Twist type message format
from sensor_msgs.msg import LaserScan
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data
from math import cos, sin, atan2, pi

import tty
import select
import sys
import termios

class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__("Obstacle_Avoidance_Node")
        self.publisher = self.create_publisher(Twist,'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan',self.process_scan, 10)
        self.linear_velocity = 0.2
        self.angular_velocity = 0.2
        self.timer = self.create_timer(0.1, self.run_loop)

    def process_scan(self,msg):
        vel = Twist()
        obstacle = []
            
        for theta, reading in enumerate(msg.ranges):
            if theta <= 60 or theta >= 300: # filter out the obstacles that are either behind or beside the neato
                if 0 < reading <= 1:# filter out the obstacles that are too far away 
                    #and the blank datapoints where no obstacles are present
                    obstacle.append(reading)

        if obstacle:
            min_obstacle = min(obstacle)# focus on the closest obstacle 

            if min_obstacle < 0.5: #if the closest obstacle is less than .5m away
                min_index = msg.ranges.index(min_obstacle)#find the angle of the obstacle

                if min_index <= 60:#if on the left side
                    vel.angular.z = -self.angular_velocity#turn right
                else:
                    vel.angular.z = self.angular_velocity #turn right
            else:
                vel.linear.x = self.linear_velocity#if all obstacke are far keep going stright
                vel.angular.z = 0.0 

        else:
            vel.linear.x = self.linear_velocity
            vel.angular.z = 0.0
        self.current_vel = vel





           # obstacles = []
            #for theta, reading in enumerate(msg.ranges):
              #   x = 3/reading*cos(theta*pi/180)
              #3   y = 3/reading*sin(theta*pi/180)
              #   if reading!=0 and reading <= 1:
               #     obstacle = [-x, -y]
               #     obstacles.append(obstacle)
          #  total_x = 0
           # total_y = 0
          #  for obstacle in obstacles:
             #   total_x += obstacle[0] 
           #     total_y += obstacle[1]
          #  resultant_angle = atan2(total_y, total_x)
          #  if resultant_angle <= 60:
          #      vel.angular.z = 0.2*
          #  elif resultant_angle >= 300:



          #  turn_time = resultant_angle/vel.linear.x
          #   


    def run_loop(self, ):
        self.publisher.publish(self.current_vel)



def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

