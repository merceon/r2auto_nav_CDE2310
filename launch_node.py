import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
servo_pin = 21
solenoid_pin = 12
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(solenoid_pin,GPIO.OUT)
p = GPIO.PWM(servo_pin, 50)
# Set servo to 90 degrees as it's starting position

#def map_value(x, in_min, in_max, out_min, out_max):
#    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

import rclpy 
from rclpy.node import Node
from std_msgs.msg import Int8

class LaunchNode(Node):
	def __init__(self):
		super().__init__('launch_topic')
		self.subscriber = self.create_subscription(Int8, 'launch_topic',self.timer_callback, 10)

	def timer_callback(self,msg):
		p.start(8)
		for x in range(3):
			for i in range(8,12):
				p.ChangeDutyCycle(i)
				time.sleep(0.1)
			for i in range(120,80,-1):
				p.ChangeDutyCycle(i/10)
				time.sleep(0.02)
			time.sleep(0.3)
			GPIO.output(solenoid_pin, GPIO.HIGH)
			time.sleep(0.5)
			GPIO.output(solenoid_pin, GPIO.LOW)
			if x==1:
				time.sleep(2)





def main(args=None):
    rclpy.init(args=args)
    Launch = LaunchNode()
    rclpy.spin(Launch)
    Launch.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()
    p.stop()

if __name__ == '__main__':
    main()



"""except KeyboardInterrupt:
	p.stop()
	GPIO.cleanup()
"""
