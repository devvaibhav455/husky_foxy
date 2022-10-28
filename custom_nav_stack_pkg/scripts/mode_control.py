#! /usr/bin/env python3

import logging
from statistics import mode
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy, NavSatFix, Imu
from geometry_msgs.msg import Twist

from geographiclib.geodesic import Geodesic


import message_filters
from pathlib import Path
import math
from transformations import euler_from_quaternion
import time

import os




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
              logger.info("Line{}: {}".format(count, line.rstrip('\n').strip('')))
              self.lat_array.append(Lines[count].rstrip('\n').split(' ')[0])
              self.long_array.append(Lines[count].rstrip('\n').split(' ')[1])
            count += 1

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

        self.mode = 'auto'
        self.imu_sub = message_filters.Subscriber(self, Imu, 'imu/data')
        self.gps_sub = message_filters.Subscriber(self, NavSatFix, 'gps/data')
        self.joy_sub = message_filters.Subscriber(self, Joy, 'joy_teleop/joy')
        
        self.velocity_publisher_ = self.create_publisher(Twist, 'husky_velocity_controller/cmd_vel_unstamped', 10)
        # Twist is a datatype for velocity
        self.move_cmd = Twist()        
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.joy_sub, self.imu_sub, self.gps_sub], 10, 0.09)
        self.ts.registerCallback(self.listener_callback)
        time.sleep(2)

    def DMS_to_decimal_format(self, lat, long): 
        '''Check for degrees, minutes, seconds format and convert to decimal'''
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

    def rotate(self,target_angle_rad,direction):
        '''Publishes twist command to rotate robot in CW or CCW direction. '''
        global bearing
        if(direction == 'CCW' or direction == 'ccw'):
                multiplier = 1  #Positive velocity means anti-clockwise (CCW) rotation (If we rotate in actual CCW direction, robot moves away from the bearing.. its a workaround)
                #print("CCW")
        else:
                multiplier = -1  #Negative velocity means clockwise (CW) rotation
                #print("CW")
    
        self.move_cmd.angular.z = multiplier * (self.kp * (abs(bearing - self.true_heading) + 0.3))
        # self.move_cmd.angular.z = multiplier * (self.kp * (abs(target_angle_rad) + 0.3))
        self.move_cmd.linear.x = abs(self.move_cmd.angular.z / 4)
        self.velocity_publisher_.publish(self.move_cmd)

    
    def listener_callback(self, joy_msg, imu_msg, gps_msg):
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
        global bearing #Bearing is the required direction towards which robot should orient
        bearing = math.atan2(X, Y)
        # rclpy.logging.get_logger('gps_callback').info('Bearing (degree):  %s' % math.degrees(bearing))

        #Setting mode control based on input from JS
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

    def move_forward(self, distance_to_move): 
        '''If distance_to_move='until_obstacle', then robot will move forward indefinitely until an obstacle is detected, otherwise give distance in meters'''
        if distance_to_move > 1:
            self.move_cmd.linear.x = 0.2
            self.move_cmd.angular.z = 0.0
            self.velocity_publisher_.publish(self.move_cmd)
        elif(distance_to_move < 1):
            global goal_number
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.velocity_publisher_.publish(self.move_cmd)
            if goal_number + 1 < len(self.lat_array):
                logger.warning("Destination #: %d reached :)", goal_number)
                goal_number+=1
            else:
                logger.warning("GPS navigation complete for all goals :)")
            if goal_number < len(self.lat_array):
                self.dest_lat, self.dest_long = self.DMS_to_decimal_format(self.lat_array[goal_number], self.long_array[goal_number])

    def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
        '''Calculate distance and azimuth between GPS points. Returns distance in meter'''
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] # access distance
        logger.warning("The distance from robot to goal #: %d is: %f m.", goal_number, distance)
        azimuth = g['azi1']
        # self.get_logger().info("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))
        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        x = adjacent = math.cos(azimuth) * hypotenuse
        y = opposite = math.sin(azimuth) * hypotenuse
        # self.get_logger().info("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))
        return distance

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
    path = str(Path(__file__).parent / "./debug.log")
    path = path.replace("install", "src")
    path = path.replace("lib/custom_nav_stack_pkg", "scripts")   
    handler = logging.FileHandler(path, 'w', 'utf-8')
    formatter = logging.Formatter(fmt='%(asctime)s %(levelname)-8s %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    #On terminal, due to some reason, only warning and above are printed, so setting the level to warning. No need to worry.
    rclpy.init(args=args)
    mode_control = ModeControl()
    rclpy.spin_once(mode_control)
    old_mode = mode_control.mode
    logger.warning('Current mode is: %s', mode_control.mode)
    while 1==1:
        mode_control.sync_done = 0
        rclpy.spin_once(mode_control)
        if mode_control.sync_done == 1:
            if(old_mode != mode_control.mode):
                logger.warning("Changed mode from: %s to %s", old_mode, mode_control.mode)
            if mode_control.mode == 'manual': #Enter into manual override mode. Complete control in hands of the user
                # Need to run node `node_teleop_twist_joy` which publishes cmd_vel to robot from JS
                pass
                # os.system("ros2 launch husky_control teleop_pub_vel.launch.py &") #No need to run this because both joy nodes are running and they override the speed command from script
                #Need to destroy other nodes here
            elif mode_control.mode == 'gps': #Enter into force GPS waypoing mode, can hit obstacles, does not call local mode when detects obstacle. Used for testing purpose
                #First try to align robot to desired heading and then move in that direction
                global bearing
                mode_control.true_heading = mode_control.mag_north_heading + mode_control.mag_declination #Robot's current heading wrt to True North; If -ve, turn right (cw); + turn left (ccw) to align it with True North
                # Calculation to get true heading within -180 to +180 degrees range
                if mode_control.true_heading < math.radians(-180):
                    mode_control.true_heading = math.radians(180 - (abs(math.degrees(mode_control.true_heading)) - 180))
                elif mode_control.true_heading > math.radians(180):
                    mode_control.true_heading = -(math.radians(180) - (mode_control.true_heading - math.radians(180)))

                # Calculations to find the angle to rotate the robot in order to align with the required heading (true heading)
                if bearing < 0:
                    if (mode_control.true_heading < bearing and mode_control.true_heading > math.radians(-180)) or (mode_control.true_heading > (math.radians(180) - abs(bearing)) and mode_control.true_heading < math.radians(180)):
                        mode_control.direction = 'cw' #Original
                        if mode_control.true_heading < 0:
                            mode_control.to_rotate = abs(bearing - mode_control.true_heading)
                        else:
                            mode_control.to_rotate = math.radians(180 - math.degrees(mode_control.true_heading) + 180 - math.degrees(abs(bearing)))
                    elif ((mode_control.true_heading < 0 and mode_control.true_heading > bearing) or (mode_control.true_heading > 0 and mode_control.true_heading < math.radians(180 - math.degrees(abs(bearing))))):
                        mode_control.direction = 'ccw' #Original
                        if mode_control.true_heading < 0:
                            mode_control.to_rotate = abs(bearing - mode_control.true_heading)
                        else:
                            mode_control.to_rotate = mode_control.true_heading + abs(bearing)
                elif bearing > 0:
                    if ((mode_control.true_heading < 0 and mode_control.true_heading > -math.radians(180 - math.degrees(bearing))) or (mode_control.true_heading > 0 and mode_control.true_heading < bearing)):
                        mode_control.direction = 'cw' #Original
                        if mode_control.true_heading < 0:
                            mode_control.to_rotate = abs(mode_control.true_heading) + bearing
                        else:
                            mode_control.to_rotate = abs(bearing - mode_control.true_heading)
                    elif ((mode_control.true_heading > bearing and mode_control.true_heading < math.radians(180)) or (mode_control.true_heading < -math.radians(180 - math.degrees(bearing)) and mode_control.true_heading > math.radians(-180))):
                        mode_control.direction = 'ccw' #Original
                        
                        if mode_control.true_heading < 0:
                            mode_control.to_rotate = math.radians(180 - abs(math.degrees(mode_control.true_heading)) + 180 - math.degrees(bearing))
                        else:
                            mode_control.to_rotate = abs(bearing - mode_control.true_heading)

                if abs(mode_control.to_rotate) > 0.06:
                    logger.warning("Info for goal #: %d, Current heading: %f°, Required heading: %f°, Need to rotate by: %f° in Direction: %s", goal_number, math.degrees(mode_control.true_heading), math.degrees(bearing), math.degrees(mode_control.to_rotate), mode_control.direction)
                    mode_control.rotate(mode_control.to_rotate, mode_control.direction)
                elif mode_control.to_rotate < 0.06: 
                    logger.warning("Robot already aligned towards goal no: %d", goal_number)
                    mode_control.distance_to_move = mode_control.calc_goal(mode_control.current_lat, mode_control.current_long, mode_control.dest_lat, mode_control.dest_long)
                    mode_control.move_forward(mode_control.distance_to_move)
                    # mode_control.destroy_timer(mode_control.timer)


            elif mode_control.mode == 'local': #Enter into force local navigation mode. Used for testing purpose
                pass
            elif mode_control.mode == 'auto': #Enter automatic mode which can auto-switch between gps and local mode
                pass

        old_mode = mode_control.mode
        # mode_control.destroy_subscription(mode_control.subscription)
            # logger.warning("While end")
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mode_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()