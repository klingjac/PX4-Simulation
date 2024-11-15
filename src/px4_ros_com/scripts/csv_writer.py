#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry, VehicleLocalPositionSetpoint
from std_msgs.msg import Bool  # Import Bool message type
import csv
from datetime import datetime
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PX4DataSubscriber(Node):

    def __init__(self):
        super().__init__('px4_data_subscriber')

        # Define QoS profile with best effort reliability
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.position = None
        self.velocity = None
        self.acceleration = None
        self.angular_rate = None
        self.position_setpoint = None
        self.recording = False  # Variable to control recording status

        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile)

        self.angular_rate_subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.angular_rate_callback,
            qos_profile)

        self.position_setpoint_subscription = self.create_subscription(
            VehicleLocalPositionSetpoint,
            '/vehicle_position_setpoint',
            self.position_setpoint_callback,
            qos_profile)

        # Subscription to the /input/done topic
        self.done_subscription = self.create_subscription(
            Bool,
            '/input/done',
            self.done_callback,
            qos_profile)

        # Open CSV file for writing (but don't write until /input/done is received)
        self.csv_file = None
        self.csv_writer = None
        self.timer = None  # Timer will be created when recording starts

    def done_callback(self, msg):
        if msg.data:  # Check if the /input/done message indicates to start recording
            self.start_recording()
        else:
            self.stop_recording()

    def start_recording(self):
        if not self.recording:
            self.recording = True
            self.get_logger().info("Starting recording.")
            # Open CSV file for writing
            self.csv_file = open('px4_data.csv', 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'position', 'velocity', 'acceleration', 'angular_rate', 'position_setpoint'])
            # Create timer for writing data to CSV at 10 Hz
            self.timer = self.create_timer(0.1, self.write_to_csv)

    def stop_recording(self):
        if self.recording:
            self.recording = False
            self.get_logger().info("Stopping recording.")
            # Cancel the timer and close the CSV file
            if self.timer:
                self.timer.cancel()
                self.timer = None
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None

    def position_callback(self, msg):
        self.position = [msg.x, msg.y, msg.z]
        self.velocity = [msg.vx, msg.vy, msg.vz]
        self.acceleration = [msg.ax, msg.ay, msg.az]

    def angular_rate_callback(self, msg):
        self.angular_rate = [msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]]

    def position_setpoint_callback(self, msg):
        self.position_setpoint = [msg.x, msg.y, msg.z]

    def write_to_csv(self):
        timestamp = datetime.now()

        # Format to show hours, minutes, seconds, and milliseconds
        timestamp = timestamp.strftime("%H:%M:%S.%f")[:-3]
        position = self.position if self.position else 'None'
        velocity = self.velocity if self.velocity else 'None'
        acceleration = self.acceleration if self.acceleration else 'None'
        angular_rate = self.angular_rate if self.angular_rate else 'None'
        position_setpoint = self.position_setpoint if self.position_setpoint else 'None'

        self.csv_writer.writerow([timestamp, position, velocity, acceleration, angular_rate, position_setpoint])

    def destroy(self):
        # Ensure recording stops and resources are released
        self.stop_recording()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)

    px4_data_subscriber = PX4DataSubscriber()

    try:
        rclpy.spin(px4_data_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        px4_data_subscriber.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
