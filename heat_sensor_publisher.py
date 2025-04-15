import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import busio
import board
import adafruit_amg88xx

i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c,addr=0x69)

class Heat_sensor_publisher(Node):
	def __init__(self):
		super().__init__("heat_sensor_publisher")
		self.publisher= self.create_publisher(Float64MultiArray,"/thermal_image",10)
		self.timer = self.create_timer(1.0, self.timer_callback)

	def timer_callback(self):
		msg = Float64MultiArray()
		data = [value for row in amg.pixels for value in row]
		msg.data = data
		self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = Heat_sensor_publisher()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
