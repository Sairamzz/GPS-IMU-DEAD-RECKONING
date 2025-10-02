import rclpy
from rclpy.node import Node
from custom_msgs.msg import GPSmsg
from std_msgs.msg import String
import csv
import os


class GPSSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            GPSmsg,
            '/gps',
            self.listener_callback,
            10)
        self.subscription

        self.csv_file_path = 'gps_data.csv'
        self.create_csv_file()

    def create_csv_file(self):
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Latitude', 'Longitude', 'Altitude', 'UTM Easting', 'UTM Northing', 'Zone','Letter'])


    def listener_callback(self, msg):

        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        utm_easting = msg.utm_easting
        utm_northing = msg.utm_northing
        zone = msg.zone
        letter = msg.letter
        
        self.get_logger().info("\n" +
                       "Latitude: " + str(latitude) + "\n" +
                       "Longitude: " + str(longitude) + "\n" +
                       "Altitude: " + str(altitude) + "\n" +
                       "UTM Easting: " + str(utm_easting) + "\n" +
                       "UTM Northing : " + str(utm_northing) + "\n" +
                       "Zone: " + str(zone) + "\n" +
                       "Letter: " + str(letter) + "\n")
        
        with open(self.csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([latitude, longitude, altitude, utm_easting, utm_northing, zone, letter ])

def main(args=None):
    rclpy.init(args=args)
    gps_subscriber = GPSSubscriber()
    rclpy.spin(gps_subscriber)
    gps_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
