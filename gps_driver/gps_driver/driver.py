#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import utm
import serial
import sys
from std_msgs.msg import Float64
from custom_msgs.msg import GPSmsg
from datetime import time

class GPSDriverNode(Node):
    def __init__(self):
        super().__init__("gps_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 4800)
        self.declare_parameter("sampling_rate", 10.0)

        custom_port = self.parse_command_line_args()

        serial_port = custom_port if custom_port else self.get_parameter("port").get_parameter_value().string_value
        serial_baud = self.get_parameter("baudrate").value
        sampling_rate = self.get_parameter("sampling_rate").value

        if serial_port is None:
            self.get_logger().error("No serial port specified.")
            rclpy.shutdown()
            return

        try:
            self.port = serial.Serial(serial_port, serial_baud, timeout=3.0)
            self.get_logger().info(f"Using GPS device on port {serial_port} at {serial_baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open port {serial_port}: {e}")
            rclpy.shutdown()
            return

        self.gps_publish = self.create_publisher(GPSmsg, "/gps", 10)

        self.get_logger().info("Publishing GPSmsgs")

        self.sleep_time = (1 / sampling_rate)
        self.timer = self.create_timer(self.sleep_time, self.timer_callback)

    def parse_command_line_args(self):
        for arg in sys.argv:
            if arg.startswith("port:"):
                return arg.split(":")[1]
        return None    
    
    def timer_callback(self):
        line = self.port.readline().decode("ascii", errors = "replace").strip()
        if line.startswith("$GPGGA"):
            self.parse_gps(line)
    
    def parse_gps(self, NMEA_gpgga_line):
        field = NMEA_gpgga_line.split(",")
        if len(field) < 15:
            self.get_logger().warn("Invalid NMEA GPGGA line ")
        
        time_stamp = field[1]

        hours = int(time_stamp[0:2])
        minutes = int(time_stamp[2:4])
        seconds = int(time_stamp[4:6])
        nano_seconds = int(float(time_stamp[6:]) * 10e9)

        gps_timestamp = time(hours, minutes, int(seconds))

        latitude_deg = field[2]
        latitude_direction = field[3]
        longitude_deg = field[4]
        longitude_direction = field[5]
        altitude = float(field[9])

        latitude = self.lat_decimal(latitude_deg, latitude_direction)
        longitude = self.long_decimal(longitude_deg, longitude_direction)

        utm_easting, utm_northing, zone, letter = utm.from_latlon(latitude, longitude)

        msg = GPSmsg()
        msg.header.stamp.sec = (hours * 3600) + (minutes * 60) + int(seconds)
        msg.header.stamp.nanosec = int(nano_seconds) 
        msg.header.frame_id = "GPS1_Frame"

        msg.latitude = latitude
        msg.longitude = longitude
        msg.altitude = altitude
        msg.utm_easting = utm_easting
        msg.utm_northing = utm_northing
        msg.zone = zone
        msg.letter = letter

        self.gps_publish.publish(msg)
    
    def lat_decimal(self, lat, direction):
        
        degrees = float(lat[0:2])
        minutes = float(lat[2:])
        decimal_value = degrees + (minutes / 60)
        if direction == "W":
            decimal_value = -decimal_value
        return decimal_value
    
    def long_decimal(self, long, direction):
        
        degrees = float(long[0:3])
        minutes = float(long[3:])
        decimal_value = degrees + (minutes / 60)
        if direction == "W":
            decimal_value = -decimal_value
        return decimal_value



def main(args=None):
    rclpy.init(args=args)
    gps_driver_node = GPSDriverNode()
    try:
        rclpy.spin(gps_driver_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_driver_node.port.close()
        gps_driver_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()

