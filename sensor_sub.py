import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np

class Heat_source_follower(Node):
    def __init__(self):
        super().__init__('heat_source_follower')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'heat_sensor',
            self.sensor_callback,
            10)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        
        self.reach = False
        self.subscription  # prevent unused variable warning
        self.robot_cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.threshold = 22.0  # adjust the value depending the enviroment 
    
    def listener_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        # replace 0's with nan
        laser_range[laser_range==0] = np.nan
        # find index with minimum value
        shortest_distance = np.nanargmin(laser_range)
        angle_degrees = shortest_distance * (18 / 11)
        if shortest_distance <= 0.5 and (angle_degrees < 5 or angle_degrees > 355):
            print("Obstacle detected, stopping robot.")
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.robot_cmd_publisher.publish(cmd)
            self.reach = True

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
            self.reach = False
        else:
            offset = col - 3.5
            if abs(offset) < 1.0:
                if self.reach:
                    self.get_logger().info("Heat source centered. Reached location.")
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                else:
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
