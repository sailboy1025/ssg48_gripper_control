#!/ucsr/bin/env python

import sys

# Output the current Python installation path
print("Current Python installation path:", sys.executable)
import rclpy
import Spectral_BLDC as Spectral
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy


class Ssg48GripperControl(Node):
    def __init__(self):
        super().__init__('ssg48_gripper_control')

        # Initialize communication with the motor
        self.communication = Spectral.CanCommunication(
            bustype='slcan', channel='/dev/ttyACM0', bitrate=1000000)
        self.motor = Spectral.SpectralCAN(
            node_id=0, communication=self.communication)
        self.upper_bound = 0.13
        self.low_bound = 0.04
        self.gripper_cmd = 0
        self.subscription_dist = self.create_subscription(
            Float64,
            'grasp_cmd',
            self.grasp_sub_cb,
            10  # QoS history depth
        )
        self.subscription_cali = self.create_subscription(
            Joy,
            'joy',
            self.cali_sub_cb,
            10
        )

    def grasp_sub_cb(self, msg):
        if msg.data != None:
            self.gripper_cmd = msg.data
            cmd_data = self.range_tracker_to_gripper(msg.data)
            self.motor.Send_gripper_data_pack(cmd_data, 50, 500, 1, 1, 0, 0)
        # self.get_logger().info(f'Gripper cmd: {gripper_cmd}')

    def cali_sub_cb(self, msg):
        if msg != None:
            if (msg.buttons[2] == 1):
                self.upper_bound = self.gripper_cmd
                self.get_logger().info(f'Calibrated, upperbound now is {self.upper_bound} meters!!')

    def range_tracker_to_gripper(self, tracker_data) -> Float64:
        # Ensure the input value is within the expected range
        if tracker_data < 0 or tracker_data > self.upper_bound:
            self.get_logger().info("Value too large!!!!!")

        # Mapping from range [0, 0.13] to [255, 0]
        mapped_value = 255 - ((tracker_data - self.low_bound) / (self.upper_bound - self.low_bound)) * 255
        # bytes must be in range(0, 256)
        if mapped_value < 0:
            mapped_value = 0
        elif mapped_value > 255:
            mapped_value = 255
        return int(mapped_value)


def main(args=None):
    rclpy.init(args=args)
    grasp_cmd_subscriber = Ssg48GripperControl()
    rclpy.spin(grasp_cmd_subscriber)
    grasp_cmd_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
