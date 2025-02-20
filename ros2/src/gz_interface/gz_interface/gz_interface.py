import json

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class GzInterface(Node):
    def __init__(self):
        super().__init__('gz_interface')
        self.publisher = self.create_publisher(
            Twist,
            '/model/vehicle/cmd_vel',
            10)
        self.get_logger().info('Teleop ready. Make sure to play the simulation.')
        while True:
            e = input(">>>>>> angular.z = <z> # rotate >>>>>> linear.x = <x> # forward/backward\n")
            self.publish(e)
       
       
    def publish(self, trigger):
        #msg = {"linear": {"x": 5.0, "y": 0.0, "z": 0.0}, "angular": {"x": 1.0, "y": 1.0, "z": 0.0}}

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        exec("msg." + trigger)
        
        self.get_logger().info("Attempting to publish: " + str(msg))
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    gz = GzInterface()

    rclpy.spin(gz)

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
