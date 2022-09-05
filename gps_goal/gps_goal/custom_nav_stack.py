#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# SOURCE: https://github.com/ros-planning/navigation2/issues/2283


from logging.handlers import BaseRotatingHandler
import time
import math

from geometry_msgs.msg import Pose, PoseStamped, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu
from sensor_msgs.msg import MagneticField
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import sys
import rclpy
import utm
from transformations import euler_from_quaternion

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from geographiclib.geodesic import Geodesic

from pathlib import Path


#ASSUMPTIONS: 1) IMU 0 orientation is TRUE NORTH | 2) IMU heading is considered as magnetometer heading (as couldn't insert magnetometer is gazebo)

class CustomNavigator(Node):
    def __init__(self):
        super().__init__(node_name='custom_navigator')
        
        self.gps_callback_done = 0
        self.gps_subscription = self.create_subscription(NavSatFix,'gps/data',self.gps_callback,10)
        self.gps_subscription  # prevent unused variable warning

        self.imu_subscription = self.create_subscription(Imu,'imu/data',self.imu_callback,10)
        self.imu_subscription  # prevent unused variable warning

        
        path = Path(__file__).parent / "./destination_lat_long.txt"
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

        self.velocity_publisher_ = self.create_publisher(Twist, 'husky_velocity_controller/cmd_vel_unstamped', 10)
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        print("Entered in class")

        for i in range(len(self.lat_array)):
            # Check for degrees, minutes, seconds format and convert to decimal
            self.dest_lat, self.dest_long = self.DMS_to_decimal_format(self.lat_array[i], self.long_array[i])

        # self.current_lat = math.radians(argv[0])
        # self.current_long = math.radians(argv[1])
        # self.dest_lat = math.radians(argv[2])
        # self.dest_long = math.radians(argv[3])
        # self.delta_long = dest_long - current_long

        # self.current_lat = math.radians(argv[0])
        # self.current_long = math.radians(argv[1])
        # self.dest_lat = math.radians(argv[2])
        # self.dest_long = math.radians(argv[3])
        # self.delta_long = dest_long - current_long

        # Reference: https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
        
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # self.target = 90
        # self.target2 = -90 
        self.kp = 0.8
        self.mag_declination = math.radians(-14.87) #14.87° W degrees is for Neumayer III Station, Antarctica (https://latitude.to/articles-by-country/aq/antarctica/27247/neumayer-station-iii)
        
        
        # time.sleep(5)
        
        # Twist is a datatype for velocity
        self.move_cmd = Twist()

        # self.gps_sub = self.create_subscription(MagneticField, '/mag', self.mag_callback)  #To be used on real robot as we couldn't get mag in gazebo
        #self.mag_heading = math.atan2(mag_y, magx)

        # self.mag_north_heading = math.radians(10) # 0 means that the magnetometer is actually pointing towards Magnetic North; +10 means that you are 10° towards East of magnetic north; i.e. need to turn 10° towards left to reach magnetic north
        
        #Reference: Calculate magnetic declination (measured wrt True North): https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination; 2022-08-26 	14.87° W  ± 0.40°  changing by  0.06° W per year
        # By convention, declination is positive when magnetic north is east of true north, and negative when it is to the west. https://upload.wikimedia.org/wikipedia/commons/thumb/c/c2/Magnetic_declination.svg/330px-Magnetic_declination.svg.png
        # self.mag_declination = math.radians(-14.87) #14.87° W degrees is for Neumayer III Station, Antarctica (https://latitude.to/articles-by-country/aq/antarctica/27247/neumayer-station-iii)
        # self.true_heading = self.mag_north_heading + self.mag_declination #Robot's current heading wrt to True North; If -ve, turn right (cw); + turn left (ccw) to align it with True North

        # self.to_rotate = bearing - self.true_heading
        # self.direction = 'ccw' if self.to_rotate < 0 else 'cw'
        # print(self.direction)
        # Publisher to send velocity commands to robot
        

        # if self.true_heading != 0:
        #     self.rotate(self.true_heading, self.direction)
        # elif self.true_heading == 0:
        #     print("Robot already aligned towards goal")

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
        rclpy.logging.get_logger('DMS_to_decimal_format').info('Given GPS goal: lat %s, long %s.' % (lat, long))
        return lat, long
    
    def imu_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        # print("Entered IMU callback")
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list) #Result in radians. -180 < Roll < 180
        self.mag_north_heading = -self.roll #ASSUMPTION 2) IMU heading is considered as magnetometer heading (as couldn't insert magnetometer is gazebo)
        # print("Finished IMU callback")

    def gps_callback(self, msg):
        self.gps_callback_done = 1
        self.current_lat = msg.latitude
        self.current_long = msg.longitude
        self.delta_long = self.dest_long - self.current_long
        X = math.cos(self.dest_lat) * math.sin(self.delta_long)
        Y = (math.cos(self.current_lat) * math.sin(self.dest_lat)) - (math.sin(self.current_lat) * math.cos(self.dest_lat) * math.cos(self.delta_long))
        global bearing
        bearing = math.atan2(X, Y)
        rclpy.logging.get_logger('gps_callback').info('Bearing (degree):  %s' % math.degrees(bearing))        # print("Entered IMU callback")
        # print("Bearing in radian: ", bearing)
        # print("Bearing in degree: ", math.degrees(bearing)) #If -45° , means need to point robot to 45° left/ west of True North

    def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
        # Calculate distance and azimuth between GPS points
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] # access distance
        self.get_logger().info("The distance from the origin to the goal is {:.3f} m.".format(distance))
        azimuth = g['azi1']
        self.get_logger().info("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        x = adjacent = math.cos(azimuth) * hypotenuse
        y = opposite = math.sin(azimuth) * hypotenuse
        self.get_logger().info("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))
        return distance
    
    def move_forward(self, distance_to_move='until_obstacle'): # If distance_to_move='until_obstacle', then robot will move forward indefinitely until an obstacle is detected, otherwise give distance in meters
        if distance_to_move == 'until_obstacle':
            #Need to make changes here to detect obstacle
            self.move_cmd.linear.x = 0.4
        else:
            


    def timer_callback(self):
        print("Entered timer callback")
        
        if self.gps_callback_done == 1:
            #First try to align robot to desired heading and then move in that direction
            global bearing
            rclpy.logging.get_logger('timer_callback').info('Entered IMU/ timer_callback') 


            self.true_heading = self.mag_north_heading + self.mag_declination #Robot's current heading wrt to True North; If -ve, turn right (cw); + turn left (ccw) to align it with True North
            if self.true_heading < math.radians(-180):
                self.true_heading = math.radians(180 - (abs(math.degrees(self.true_heading)) - 180))
            elif self.true_heading > math.radians(180):
                self.true_heading = -(math.radians(180) - (self.true_heading - math.radians(180)))


            if bearing < 0:
                if (self.true_heading < bearing and self.true_heading > math.radians(-180)) or (self.true_heading > (math.radians(180) - abs(bearing)) and self.true_heading < math.radians(180)):
                    self.direction = 'cw'
                    if self.true_heading < 0:
                        self.to_rotate = abs(bearing - self.true_heading)
                    else:
                        self.to_rotate = math.radians(180 - math.degrees(self.true_heading) + 180 - math.degrees(abs(bearing)))
                elif ((self.true_heading < 0 and self.true_heading > bearing) or (self.true_heading > 0 and self.true_heading < math.radians(180 - math.degrees(abs(bearing))))):
                    self.direction = 'ccw'
                    if self.true_heading < 0:
                        self.to_rotate = abs(bearing - self.true_heading)
                    else:
                        self.to_rotate = self.true_heading + abs(bearing)
            elif bearing > 0:
                if ((self.true_heading < 0 and self.true_heading > -math.radians(180 - math.degrees(bearing))) or (self.true_heading > 0 and self.true_heading < bearing)):
                    self.direction = 'cw'
                    if self.true_heading < 0:
                        self.to_rotate = abs(self.true_heading) + bearing
                    else:
                        self.to_rotate = abs(bearing - self.true_heading)
                elif ((self.true_heading > bearing and self.true_heading < math.radians(180)) or (self.true_heading < -math.radians(180 - math.degrees(bearing)) and self.true_heading > math.radians(-180))):
                    self.direction = 'ccw'
                    if self.true_heading < 0:
                        self.to_rotate = math.radians(180 - abs(math.degrees(self.true_heading)) + 180 - math.degrees(bearing))
                    else:
                        self.to_rotate = abs(bearing - self.true_heading)


            rclpy.logging.get_logger('timer_callback').info('Need to rotate robot by (degree):  %s ° in %s ' % (math.degrees(self.to_rotate), self.direction))        # print("Entered IMU callback")
            # print("Need to rotate robot by", math.degrees(self.to_rotate), "° in ", self.direction)
            # self.to_rotate = bearing - self.true_heading
            # self.direction = 'ccw' if self.to_rotate < 0 else 'cw'
            # print(math.degrees(self.roll), math.degrees(self.pitch), math.degrees(self.yaw))
            # print(self.roll, self.pitch, self.yaw)
            # print(math.degrees(self.roll)) #Roll is providing the current angular rotation surprisingly

            if abs(self.to_rotate) > 0.06:
                self.rotate(self.to_rotate, self.direction)
            elif self.to_rotate < 0.06:
                print("Robot already aligned towards goal")
                self.destroy_timer(self.timer)
                print("Finished timer callback")
                self.distance_to_move = self.calc_goal(self.current_lat, self.current_long, self.dest_lat, self.dest_long)
                self.move_forward

            


            

    def rotate(self,target_angle_rad,direction):
        print("Entered rotate function")
        global bearing
        if(direction == 'ccw'):
                multiplier = 1  #Positive velocity means anti-clockwise (CCW) rotation
                print("CCW")
        else:
                multiplier = -1  #Negative velocity means clockwise (CW) rotation
                print("CW")
        #print("Reached 1")
        # target_angle_rad = multiplier * target_angle_deg*math.pi/180
        #print("Angular z is: ",self.move_cmd.angular.z)
        # publish the velocit
        rotation_complete = 0
        print("Bearing (in degree): ", math.degrees(bearing))
        print("True heading (in degree): ", math.degrees(self.true_heading))
        print("DEGREE: Need_to_rorate={} current:{}", math.degrees(target_angle_rad), math.degrees(self.true_heading))
        print("RADIAN: Need_to_rorate={} current:{}", target_angle_rad, self.true_heading)
        
        # if abs(target_angle_rad - self.roll) < 0.06:
            # rotation_complete = 1
            # print("Met the target rotation approximately")
            # self.move_cmd.angular.z = 0.0
            # self.velocity_publisher_.publish(self.move_cmd)
            # self.destroy_timer(self.timer)
            # # time.sleep(8)
            # return None
        # else:
        print("Reached 2")
        # print("target={} current:{}", target_angle_rad,self.roll)
        self.move_cmd.angular.z = multiplier * (self.kp * (abs(bearing - self.true_heading) + 0.3))
        # self.move_cmd.angular.z = multiplier * 0.5
        print(self.move_cmd.angular.z)
        self.velocity_publisher_.publish(self.move_cmd)
        time.sleep(1)
        print("Finished single execution of rotate function")
            

def main(argv=sys.argv[1:]):

    # print(argv[0], argv[1])
    rclpy.init()


    
    custom_navigator = CustomNavigator()
    rclpy.spin(custom_navigator)

    
    

if __name__ == '__main__':
  main()