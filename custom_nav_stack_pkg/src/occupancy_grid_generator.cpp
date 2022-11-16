#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <vector>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

//          nav_msgs/msg/OccupancyGrid sample message		|||		            sensor_msgs/msg/LaserScan sample message
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// header:				                                    |||		header:
//   stamp:				                                    |||		  stamp:
//     sec: 0				                                |||		    sec: 3001
//     nanosec: 0				                            |||		    nanosec: 544000000
//   frame_id: map				                            |||		  frame_id: velodyne
// info:				                                    |||		angle_min: -3.1414999961853027
//   map_load_time:				                            |||		angle_max: 3.1414999961853027
//     sec: 0				                                |||		angle_increment: 0.008700000122189522
//     nanosec: 0				                            |||		time_increment: 0.0
//   resolution: 0.05000000074505806				        |||		scan_time: 0.33329999446868896
//   width: 720				                                |||		range_min: 0.30000001192092896
//   height: 720				                            |||		range_max: 20.0
//   origin:				                                |||		ranges: '<sequence type: float, length: 723>'
//     position:				                            |||		intensities: '<sequence type: float, length: 0>'
//       x: -17.91928748179766				                |||		
//       y: -17.99988299188389				                |||		
//       z: 0.0				                                |||		
//     orientation:				                            |||		
//       x: 0.0				                                |||		
//       y: 0.0				                                |||		
//       z: 0.0				                                |||		
//       w: 1.0				                                |||		
// data: '<sequence type: int8, length: 518400>'			|||		


// clear && colcon build --packages-select custom_nav_stack_pkg --symlink-install && source install/local_setup.bash && ros2 run custom_nav_stack_pkg occupancy_grid_generator

std::vector<int> X;
std::vector<int> Y;

class LaserScanSubscriber : public rclcpp::Node
{
    struct Config {
        int grid_size;
        float grid_cell_length;
        int test_x1;
        int test_x2;
        int test_y1;
        int test_y2;
        };
    Config config;



    public:
        
        // Ref: https://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html
        // void linev6(int x1, int y1, int x2, int y2) const {
        //     int dx  = x2 - x1,
        //         dy  = y2 - y1,
        //         y   = y1,
        //         eps = 0;
            
        //     // std::vector<int> Y = {1, 3, 2, 4};
        //     // plt::plot(Y);
            
            
        //     for ( int x = x1; x <= x2; x++ )  {
        //         // plt::plot(x, y);
        //         // s.Plot(x,y,colour);
        //         X.push_back(x);
        //         Y.push_back(y);
        //         // plt::plot(Y);
        //         eps += dy;
        //         if ( (eps << 1) >= dx )  {
        //         y++;  eps -= dx;
        //         }
        //     }
        // }

        // Ref: http://tech-algorithm.com/articles/drawing-line-using-bresenham-algorithm/
        void line(int x,int y,int x2, int y2) const {
            int w = x2 - x ;
            int h = y2 - y ;
            int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0 ;
            if (w<0){dx1 = -1 ;} else if (w>0) {dx1 = 1 ;}
            if (h<0) {dy1 = -1 ;} else if (h>0) {dy1 = 1 ;}
            if (w<0) {dx2 = -1 ;} else if (w>0) {dx2 = 1 ;}
            int longest = abs(w) ;
            int shortest = abs(h) ;
            if (!(longest>shortest)) {
                longest = abs(h) ;
                shortest = abs(w) ;
                if (h<0) {dy2 = -1 ;} else if (h>0) {dy2 = 1 ;}
                dx2 = 0 ;            
            }
            int numerator = longest >> 1 ;
            for (int i=0;i<=longest;i++) {
                X.push_back(x);
                Y.push_back(y);
                numerator += shortest ;
                if (!(numerator<longest)) {
                    numerator -= longest ;
                    x += dx1 ;
                    y += dy1 ;
                } else {
                    x += dx2 ;
                    y += dy2 ;
                }
            }
        }
        
        LaserScanSubscriber() //Constructor which has the same name as that of class followed by a parentheses. A constructor in C++ is a special method that is automatically called when an object of a class is created.
        : Node("laserscan_subscriber")
        {
        //Read config from file
        std::ifstream config_file_path("/home/dev/husky_ws/src/custom_nav_stack_pkg/src/config.txt");
        std::string line;
        while (std::getline(config_file_path, line)) {
            std::istringstream sin(line.substr(line.find("=") + 1));
            if (line.find("grid_size") == 0)
                sin >> config.grid_size;
            else if (line.find("grid_cell_length") == 0)
                sin >> config.grid_cell_length;
            else if (line.find("test_x1") == 0)
                sin >> config.test_x1;
            else if (line.find("test_x2") == 0)
                sin >> config.test_x2;
            else if (line.find("test_y1") == 0)
                sin >> config.test_y1;
            else if (line.find("test_y2") == 0)
                sin >> config.test_y2;
        }
        std::cout << config.grid_size << '\n';
        
        plt::figure();
        // std::vector<double> y = {1, 3, 2, 4};
        // plt::plot(y);
        // // plt::savefig("minimal.pdf");
        // plt::show();
        
        publisher_occ_grid_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_map", 10);
        subscription_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>
        ("scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanSubscriber::laserscan_callback, this, std::placeholders::_1));
        }

    private:
        void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_ptr) const
        {
            std::cout << "############## Callback function starts ##############" << std::endl;
            RCLCPP_INFO(this->get_logger(), "I heard at -180: '%f'", msg_ptr->ranges[0]);
            // linev6(0,0,2,4);
            // linev6(4,2,0,0);
            // line(4,0,4,2);
            line(config.test_x1, config.test_y1, config.test_x2, config.test_y2);
            std::cout << "X size: " << X.size() << " ; Y size: " << Y.size() << std::endl;
            std::cout << "X is: "; 
            for (auto i: X) { std::cout << i << " " ;} ;
            std::cout << " ; Y is: "; 
            for (auto i: Y) { std::cout << i << " " ;} ;

            plt::plot(X,Y);
            plt::show();
            std::cout << "\n############## Callback function ends ##############" << std::endl;
        }
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_occ_grid_map;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laserscan;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanSubscriber>());
  rclcpp::shutdown();
  return 0;
}