#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import math
from time import sleep

from hi221_imu.hipnuc_module import *

# from time import time
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from transforms3d.euler import euler2quat as quaternion_from_euler
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


degrees2rad = math.pi / 180.0

WAIT_TIME = 10


class Hi221ImuDriver(Node):
    def __init__(self):
        super().__init__('imu_node')
        # We only care about the most recent measurement, i.e. queue_size=1
        pub_imu = self.create_publisher(Imu, 'imu', 1)

        diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 1)
        diag_pub_time = self.get_clock().now()

        imu_msg = Imu()
        # TODO arrays not supported as parameter type ROS2
        imu_msg.orientation_covariance = [0.0025, 0.0, 0.0,
                                          0.0, 0.0025, 0.0,
                                          0.0, 0.0, 0.0025]
        # self.declare_parameter('orientation_covariance').value
        imu_msg.angular_velocity_covariance = [0.002, 0.0, 0.0,
                                               0.0, 0.002, 0.0,
                                               0.0, 0.0, 0.002]
        # self.declare_parameter('velocity_covariance').value
        imu_msg.linear_acceleration_covariance = [0.04, 0.0, 0.0,
                                                  0.0, 0.04, 0.0,
                                                  0.0, 0.0, 0.04]
        # self.declare_parameter('acceleration_covariance').value
        imu_msg.header.frame_id = self.declare_parameter('frame_header', 'base_imu_link').value

        publish_magnetometer = self.declare_parameter('publish_magnetometer', False).value

        if publish_magnetometer:
            pub_mag = self.create_publisher(MagneticField, 'mag', 1)
            mag_msg = MagneticField()
            mag_msg.magnetic_field_covariance = [0.00, 0.0, 0.0,
                                                 0.0, 0.00, 0.0,
                                                 0.0, 0.0, 0.00]
            # self.declare_parameter('magnetic_field_covariance').value
            mag_msg.header.frame_id = self.get_parameter_or('frame_header', 'base_imu_link').value
            # should a separate diagnostic for the Magnetometer be done?

        port = self.declare_parameter('port', '/dev/ttyUSB0').value

        # read calibration parameters
        self.calib_dict = {}

        # accelerometer
        self.calib_dict["accel_x_min"] = self.declare_parameter('accel_x_min', -250.0).value
        self.calib_dict["accel_x_max"] = self.declare_parameter('accel_x_max', 250.0).value
        self.calib_dict["accel_y_min"] = self.declare_parameter('accel_y_min', -250.0).value
        self.calib_dict["accel_y_max"] = self.declare_parameter('accel_y_max', 250.0).value
        self.calib_dict["accel_z_min"] = self.declare_parameter('accel_z_min', -250.0).value
        self.calib_dict["accel_z_max"] = self.declare_parameter('accel_z_max', 250.0).value

        # magnetometer
        self.calib_dict["magn_x_min"] = self.declare_parameter('magn_x_min', -600.0).value
        self.calib_dict["magn_x_max"] = self.declare_parameter('magn_x_max', 600.0).value
        self.calib_dict["magn_y_min"] = self.declare_parameter('magn_y_min', -600.0).value
        self.calib_dict["magn_y_max"] = self.declare_parameter('magn_y_max', 600.0).value
        self.calib_dict["magn_z_min"] = self.declare_parameter('magn_z_min', -600.0).value
        self.calib_dict["magn_z_max"] = self.declare_parameter('magn_z_max', 600.0).value
        self.calib_dict["magn_use_extended"] = self.declare_parameter(
            'calibration_magn_use_extended', False).value
        self.calib_dict["magn_ellipsoid_center"] = self.declare_parameter('magn_ellipsoid_center',
                                                                          [0, 0, 0]).value
        # TODO Array of arrays not supported as parameter type ROS2
        self.calib_dict["magn_ellipsoid_transform"] = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        # self.declare_parameter('magn_ellipsoid_transform',[[0, 0, 0], [0, 0, 0], [0, 0, 0]]).value

        # gyroscope
        self.calib_dict["gyro_average_offset_x"] = self.declare_parameter('gyro_average_offset_x',
                                                                          0.0).value
        self.calib_dict["gyro_average_offset_y"] = self.declare_parameter('gyro_average_offset_y',
                                                                          0.0).value
        self.calib_dict["gyro_average_offset_z"] = self.declare_parameter('gyro_average_offset_z',
                                                                          0.0).value

        imu_yaw_calibration = self.declare_parameter('imu_yaw_calibration', 0.0).value

        # Check your COM port and baud rate
        self.get_logger().info(f"Hi221 IMU -> Opening {port}...")

        roll = 0
        pitch = 0
        yaw = 0
        accel_factor = 9.806 / 256.0  # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.

        ### configure board ###
        m_IMU = hipnuc_module(port=port)

        self.get_logger().info("Hi221 IMU up and running")

        while rclpy.ok():
            data = m_IMU.get_module_data(WAIT_TIME)

            device_idx = 0
            if len(data["euler"]) < 1:
                continue
            euler_data = data["euler"][device_idx]
            # quat_data = data["quat"][device_idx]
            acc_data = data["acc"][device_idx]
            gyr_data = data["gyr"][device_idx]
            mag_data = data["mag"][device_idx]

            # in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
            # yaw_deg = -float(words[0])
            yaw_deg = euler_data["Yaw"]
            if yaw_deg > 180.0:
                yaw_deg = yaw_deg - 360.0
            if yaw_deg < -180.0:
                yaw_deg = yaw_deg + 360.0
            yaw = yaw_deg * degrees2rad
            # in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
            pitch = -euler_data["Pitch"] * degrees2rad
            roll = euler_data["Roll"] * degrees2rad

            # Publish message
            # AHRS firmware accelerations are negated
            # This means y and z are correct for ROS, but x needs reversing
            imu_msg.linear_acceleration.x = -acc_data["X"] * accel_factor
            imu_msg.linear_acceleration.y = acc_data["Y"] * accel_factor
            imu_msg.linear_acceleration.z = acc_data["Z"] * accel_factor            

            imu_msg.angular_velocity.x = gyr_data["X"]
            # in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
            imu_msg.angular_velocity.y = -gyr_data["Y"]
            # in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
            imu_msg.angular_velocity.z = -gyr_data["Z"]

            if publish_magnetometer:
                # according to line 178 the units published of the mag are mGauss
                # REP103 specifys the units of magnetic field to be Teslas
                # The axis of the MPU magnetometer are X forward, Y right and Z down
                #  when the chip is facing forward, in the sparkfun board, the chip is facing the left side
                # but Sparkfun the firmware interchanges x and y and changes the sign of y
                # so to get it to REP103 we need to make X and Z negative
                # The magn frames form sparkfun can be seen in line 178 from Sensors.ino
                mag_msg.magnetic_field.x = mag_data["X"] * 1e-7
                mag_msg.magnetic_field.y = -mag_data["Y"] * 1e-7
                mag_msg.magnetic_field.z = -mag_data["Z"] * 1e-7
                # check frame orientation and units

            q = quaternion_from_euler(roll, pitch, yaw)
            imu_msg.orientation.x = q[0]
            imu_msg.orientation.y = q[1]
            imu_msg.orientation.z = q[2]
            imu_msg.orientation.w = q[3]
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            pub_imu.publish(imu_msg)

            if publish_magnetometer:
                mag_msg.header.stamp = imu_msg.header.stamp
                pub_mag.publish(mag_msg)

            if (diag_pub_time < self.get_clock().now()):
                diag_arr = DiagnosticArray()
                diag_arr.header.stamp = self.get_clock().now().to_msg()
                diag_arr.header.frame_id = '1'
                diag_msg = DiagnosticStatus()
                diag_msg.name = 'Hi221_Imu'
                diag_msg.level = DiagnosticStatus.OK
                diag_msg.message = 'Received AHRS measurement'

                for obj in [{'roll (deg)': str(roll * (180.0 / math.pi))},
                             {'pitch (deg)': str(pitch * (180.0 / math.pi))},
                             {'yaw (deg)': str(yaw * (180.0 / math.pi))}]:
                    kv = KeyValue()
                    for k, v in obj.items():
                        kv.key = k
                        kv.value = v
                        diag_msg.values.append(kv)

                diag_arr.status.append(diag_msg)
                diag_pub.publish(diag_arr)

    def send_command(self, serial_instance, command, value=None):
        if value is None:
            cmd = command + chr(13)
        else:
            cmd = command + str(value) + chr(13)
        self.get_logger().info(f"Razor IMU -> Sending: {cmd}")
        expected_len = len(cmd)
        res = serial_instance.write(str.encode(cmd))
        if res != expected_len:
            self.get_logger().error(
                f"Hi221 IMU -> Expected serial command len ({str(expected_len)}) "
                )
        sleep(0.05)  # Don't spam serial with too many commands at once


def main(args=None):
    rclpy.init(args=args)
    node = Hi221ImuDriver()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
