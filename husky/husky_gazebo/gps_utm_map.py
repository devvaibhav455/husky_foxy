#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import utm


class GPS_UTM_map(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps/data',
            self.gps_callback,
            10)
        self.subscription  # prevent unused variable warning

    def gps_callback(self, data):
        # self.get_logger().info('I heard: "%s"' % data)
        print("gps subscriber created successfully")                                                   
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.utm_array = utm.from_latlon(self.latitude, self.longitude)
        # self.utm_array[0] = self.utm_array[0]
        print(self.latitude, self.longitude, self.utm_array)


def main(args=None):
    rclpy.init(args=args)

    gps_subscriber = GPS_UTM_map()
    
    # rclpy.spin_once(gps_subscriber)
    rclpy.spin_once(gps_subscriber)
    
    current_utmE = gps_subscriber.utm_array[0]
    current_utmN = gps_subscriber.utm_array[1]
    print(current_utmE, current_utmN)
    spawn_lat = -3.902358194307438e-06
    spawn_long = -4.315430830891546e-07
    spawn_utmE = 833978.5087343829
    spawn_utmN = 9999999.56807624
    current_x = current_utmE - spawn_utmE #0.002633306197822094
    current_y = current_utmN - spawn_utmN #0.0011600330471992493
    print(current_x, current_y)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()