#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial
import math
import sys
from custom_msgs_imu.msg import IMUmsg
from time import sleep

class IMUDriverNode(Node):
    def __init__(self):
        super().__init__("imu_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("output_frequency", 40)

        custom_port = self.parse_command_line_args()
        
        if custom_port:   
            serial_port = custom_port
        else:
            serial_port = self.get_parameter("port").get_parameter_value().string_value
        
        if serial_port is None:
            self.get_logger().error("No serial port specified.")
            rclpy.shutdown()
            return
        
        serial_baud  = self.get_parameter("baudrate").value
        self.output_frequency = self.get_parameter("output_frequency").value
        

        try:
            self.port = serial.Serial(serial_port, serial_baud, timeout=3.0)
            self.get_logger().info(f"Using IMU device on port {serial_port} at {serial_baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open port {serial_port}: {e}")
            rclpy.shutdown()
            return
        
        self.imu_output_frequency()
        
        self.imu_publish = self.create_publisher(IMUmsg, "/imu", 10)
       
        self.get_logger().info("Publishing GPSmsgs")

        self.sleep_time = (1 / self.output_frequency)
        self.timer = self.create_timer(self.sleep_time, self.timer_callback)
        
    def parse_command_line_args(self):
        for arg in sys.argv:
            if arg.startswith("port:"):
                return arg.split(":")[1]
        return None
    
    def imu_output_frequency(self):
        command = "$VNWRG,07,40\r\n"
        self.port.write(command.encode("ascii"))
        sleep(1.0)

    def timer_callback(self):
        line = self.port.readline().decode("ascii", errors = "replace").strip()
        if line.startswith("$VNYMR"):
            try:
                self.parse_imu(line)
            except:
                print("Error",line)
        else:
            self.get_logger().warn("Unrecognized line format")
    
    def parse_imu(self,NMEA_line):
        field = NMEA_line.split(",")
        if len(field) != 13:
            self.get_logger().warn("Invalid line ")
            return
        
        yaw = float(field[1])
        pitch = float(field[2])
        roll = float(field[3])
        mag_x = float(field[4]) * 0.0001
        mag_y = float(field[5]) * 0.0001
        mag_z = float(field[6]) * 0.0001
        acc_x = float(field[7])
        acc_y = float(field[8])
        acc_z = float(field[9])
        gyro_x = float(field[10])
        gyro_y = float(field[11])
        gyro_z_str = field[12].split('*')[0]
        gyro_z = float(gyro_z_str)

        qx, qy, qz, qw = self.euler_to_quaternion_conversion(roll, pitch, yaw)

        msg = IMUmsg()
        msg.header.frame_id = "IMU1_Frame"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.imu.orientation.x = qx
        msg.imu.orientation.y = qy
        msg.imu.orientation.z = qz
        msg.imu.orientation.w = qw

        msg.imu.linear_acceleration.x = acc_x
        msg.imu.linear_acceleration.y = acc_y
        msg.imu.linear_acceleration.z = acc_z

        msg.imu.angular_velocity.x = gyro_x
        msg.imu.angular_velocity.y = gyro_y
        msg.imu.angular_velocity.z = gyro_z

        msg.mag_field.magnetic_field.x = mag_x
        msg.mag_field.magnetic_field.y = mag_y
        msg.mag_field.magnetic_field.z = mag_z

        self.imu_publish.publish(msg)
    
    def euler_to_quaternion_conversion(self,roll,pitch,yaw):
        r = float(math.radians(roll))
        p = float(math.radians(pitch))
        y = float(math.radians(yaw))

        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return qx, qy, qz, qw
        

        
def main(args=None):
    rclpy.init(args=args)
    imu_driver_node = IMUDriverNode()
    try:
        rclpy.spin(imu_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_driver_node.port.close()
        imu_driver_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()

