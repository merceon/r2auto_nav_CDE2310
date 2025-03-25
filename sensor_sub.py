import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

class Heat_source_follower(Node):
    def __init__(self):
        super().__init__('heat_source_follower')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'heat_sensor',
            self.sensor_callback,
            10)
        self.robot_cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.threshold = 25.0  # adjust the value depending the enviroment 

    def sensor_callback(self, msg):
        data = msg.data
        if len(data) != 64:
            self.get_logger().warn("Received data does not contain 64 elements.") # for error checking
            return
        max_value = max(data)
        max_index = data.index(max_value)
        row = max_index // 8
        col = max_index % 8
        
        self.get_logger().info(f"Max temperature: {max_value:.2f} at row {row}, col {col}")
        
        cmd = Twist()
        if max_value < self.threshold:
            self.get_logger().info("No significant heat source detected. Stopping.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            offset = col - 3.5
            if abs(offset) < 1.0:
                self.get_logger().info("Heat source centered. Moving forward.")
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
            else:
                self.get_logger().info("Heat source off center. Turning.")
                cmd.linear.x = 0.0
                cmd.angular.z = -0.1 * offset # value needs to be adjusted acordingly

        self.robot_cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Heat_source_follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
