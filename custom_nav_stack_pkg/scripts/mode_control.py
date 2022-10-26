#! /usr/bin/env python3

import logging
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix, Imu

import message_filters
from pathlib import Path
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import math
from transformations import euler_from_quaternion
import time


logger = logging.getLogger("my_logger")

class ModeControl(Node):

    def __init__(self):
        super().__init__('mode_control')
        self.analog_axes = [0 , 1, 3, 4]
        self.button_gps_mode = 4
        self.button_local_mode = 5
        self.button_manual_mode = 6 # Back button
        self.button_auto_mode = 0 #A or green button

        path = str(Path(__file__).parent / "./destination_lat_long.txt")
        path = path.replace("install", "src")
        path = path.replace("lib/custom_nav_stack_pkg", "scripts")

        f = open(path, "r")
        Lines = f.readlines()
        self.lat_array =[]
        self.long_array = []
        count = 0
        # Strips the newline character
        for line in Lines:
            if not (Lines[count].startswith('#') or Lines[count].startswith('\n') or Lines[count].startswith(' ')) :
              print("Line{}: {}".format(count, line.rstrip('\n').strip('')))
              self.lat_array.append(Lines[count].rstrip('\n').split(' ')[0])
              self.long_array.append(Lines[count].rstrip('\n').split(' ')[1])
            count += 1

        print(self.lat_array, self.long_array)
        global goal_number
        goal_number = 0
        self.dest_lat, self.dest_long = self.DMS_to_decimal_format(self.lat_array[goal_number], self.long_array[goal_number])

        # Reference: https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/  
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.kp = 0.8
        # self.mag_declination = math.radians(-14.87) #14.87° W degrees is for Neumayer III Station, Antarctica (https://latitude.to/articles-by-country/aq/antarctica/27247/neumayer-station-iii)
        self.mag_declination = math.radians(-4.23) # 4.23° W at 0 lat/ long
        self.imu_heading_offset = math.radians(4.235193576741463 - 90) #At robot's initial spawn position, IMU provides ~ 4.23° heading  which is towards West. Need to account for this offset. In practice, it shouldn't matter as heading will be calculated from magnetometer readings which are wrt real directions.


        self.imu_sub = message_filters.Subscriber(self, Imu, 'imu/data')
        self.gps_sub = message_filters.Subscriber(self, NavSatFix, 'gps/data')
        self.joy_sub = message_filters.Subscriber(self, Joy, 'joy_teleop/joy')
        self.chatter_sub = message_filters.Subscriber(self, String, 'chatter')
        # self.ts = message_filters.ApproximateTimeSynchronizer([self.chatter_sub, self.imu_sub, self.gps_sub], 100, 1)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.joy_sub, self.imu_sub, self.gps_sub], 10, 0.09)
        self.ts.registerCallback(self.listener_callback)
        time.sleep(2)

        print("Init ended")

    def DMS_to_decimal_format(self, lat, long):
            # Check for degrees, minutes, seconds format and convert to decimal
            if ',' in lat:
              degrees, minutes, seconds = lat.split(',')
              degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
              if lat[0] == '-': # check for negative sign
                minutes = -minutes
                seconds = -seconds
              lat = degrees + minutes/60 + seconds/3600
            if ',' in long:
              degrees, minutes, seconds = long.split(',')
              degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
              if long[0] == '-': # check for negative sign
                minutes = -minutes
                seconds = -seconds
              long = degrees + minutes/60 + seconds/3600

            lat = float(lat)
            long = float(long)
            # rclpy.logging.get_logger('DMS_to_decimal_format').info('Given GPS goal: lat %s, long %s.' % (lat, long))
            return lat, long

    def listener_callback(self, joy_msg, imu_msg, gps_msg):
        # self.get_logger().info('I heard: "%s"' % joy_msg)
        self.sync_done = 1
        
        #Extracting data from IMU sensor
        orientation_q = imu_msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list) #Result in radians. -180 < Roll < 180
        self.mag_north_heading = -self.roll + self.imu_heading_offset #ASSUMPTION 2) IMU heading is considered as magnetometer heading (as couldn't insert magnetometer is gazebo)

        #Extracting data from GPS sensor
        self.current_lat = gps_msg.latitude
        self.current_long = gps_msg.longitude
        self.delta_long = self.dest_long - self.current_long
        X = math.cos(self.dest_lat) * math.sin(self.delta_long)
        Y = (math.cos(self.current_lat) * math.sin(self.dest_lat)) - (math.sin(self.current_lat) * math.cos(self.dest_lat) * math.cos(self.delta_long))
        global bearing
        bearing = math.atan2(X, Y)
        rclpy.logging.get_logger('gps_callback').info('Bearing (degree):  %s' % math.degrees(bearing))


        if(joy_msg.buttons[self.button_gps_mode] == 1):
            self.mode = 'gps'
        elif(joy_msg.buttons[self.button_local_mode] == 1):
            self.mode = 'local'
        elif(joy_msg.buttons[self.button_auto_mode] == 1):
            self.mode = 'auto'
        elif(joy_msg.buttons[self.button_manual_mode] == 1 or joy_msg.axes[self.analog_axes[0]] != 0 or joy_msg.axes[self.analog_axes[1]] != 0 or joy_msg.axes[self.analog_axes[2]] != 0 or joy_msg.axes[self.analog_axes[3]] != 0):
            self.mode = 'manual'
        else:
            self.mode = 'auto'
        # self.destroy_subscription(self.subscription)


def main(args=None):
    
    # logging.basicConfig(
    #     filename='debug.log',
    #     filemode='w',
    #     # encoding='utf-8',
    #     level=logging.DEBUG,
    #     # level=logging.INFO,
    #     format='%(asctime)s %(levelname)08s %(name)s %(message)s',
    # )

    logger.setLevel(logging.DEBUG) #Levels: https://docs.python.org/3/howto/logging.html : The default level is WARNING, which means that only events of this level and above will be tracked, unless the logging package is configured to do otherwise. To get all logs, set level to DEBUG (lowest level). DEBUG < INFO < WARNING < ERROR < CRITICAL
    
        # Uncomment these lines to get debug logs in the file. Keep them commented to see info on terminal
    # handler = logging.FileHandler('debug.log', 'w', 'utf-8')
    # formatter = logging.Formatter(fmt='%(asctime)s %(levelname)-8s %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    # handler.setFormatter(formatter)
    # logger.addHandler(handler)
    
    #On terminal, due to some reason, only warning and above are printed, so setting the level to warning. No need to worry.
    rclpy.init(args=args)
    mode_control = ModeControl()
    rclpy.spin_once(mode_control)
    if mode_control.sync_done == 1:
        old_mode = mode_control.mode
    while 1==1:
        mode_control.sync_done = 0
        rclpy.spin_once(mode_control)
        if(old_mode != mode_control.mode):
            logger.warning("Changed mode from: %s to %s", old_mode, mode_control.mode)
        if mode_control.sync_done == 1:
            logger.warning('Current mode is: %s', mode_control.mode)
            if mode_control.mode == 'gps': #Enter into GPS navigation mode
                pass

        old_mode = mode_control.mode
        # mode_control.destroy_subscription(mode_control.subscription)
            # print("While end")
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mode_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()