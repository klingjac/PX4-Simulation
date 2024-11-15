#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

class PressureSensorNode(Node):
    def __init__(self):
        super().__init__('pressure_sensor_node')
        self.publisher_ = self.create_publisher(Test, '/fmu/in/Test', 10)  # Change topic to your uORB topic
        self.timer = self.create_timer(0.5, self.read_and_publish_pressure)
        
        # Initialize ADS1115
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.ads = ADS.ADS1115(self.i2c)
        self.ads.gain = 1
        self.channel = AnalogIn(self.ads, ADS.P0)

    def read_and_publish_pressure(self):
        voltage = self.channel.voltage
        pressure = self.voltage_to_pressure(voltage)
        
        msg = Test()  # Use your custom message here
        msg.pressure = pressure
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Pressure: %s kPa' % msg.pressure)

    def voltage_to_pressure(self, voltage):
        # Convert voltage to pressure (Example conversion, adjust based on calibration)
        v_offset = 2.5
        v_scale = 1
        pressure = (voltage - v_offset) * 1000 / v_scale
        return pressure

def main(args=None):
    rclpy.init(args=args)
    node = PressureSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
