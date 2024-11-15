#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Assuming you are publishing raw pressure values as floats
import smbus2
import time

class PressureSensorNode(Node):
    def __init__(self):
        super().__init__('pressure_sensor_node')
        self.publisher_ = self.create_publisher(Float32, '/fmu/in/Test', 10)
        self.timer = self.create_timer(0.5, self.read_and_publish_pressure)

        # ADS1115 default address and I2C bus
        self.ads_address = 0x48
        self.bus = smbus2.SMBus(1)  # 1 indicates /dev/i2c-1
        self.config = [0xC3, 0x83]  # Config for single-ended reading from AIN0 with 4.096V and 128SPS

    def read_and_publish_pressure(self):
        # Write config to start a single conversion
        self.bus.write_i2c_block_data(self.ads_address, 0x01, self.config)
        time.sleep(0.01)  # Short delay to allow conversion to complete
        
        # Read conversion result
        data = self.bus.read_i2c_block_data(self.ads_address, 0x00, 2)
        val = data[0] << 8 | data[1]
        if val > 0x7FFF:
            val -= 0x10000  # Convert to 16-bit signed
        
        # Calculate voltage based on ADS1115 config (assuming PGA = +/-4.096V)
        voltage = val * 4.096 / 32768.0
        
        # Convert voltage to pressure
        pressure = self.voltage_to_pressure(voltage)
        
        msg = Float32()
        msg.data = pressure
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Pressure: {pressure} kPa')

    def voltage_to_pressure(self, voltage):
        # Example conversion, adjust based on calibration
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
