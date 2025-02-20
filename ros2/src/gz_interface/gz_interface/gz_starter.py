import rclpy
from rclpy.node import Node

class GzInterface(Node):
    def __init__(self):
        super().__init__('gz_interface')
        self.get_logger().info("GZ interface started...")
        

def main(args=None):
    rclpy.init(args=args)

    gz = GZInterface()

    rclpy.spin(gz)

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
