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

class LaserScanSubscriber : public rclcpp::Node
{
    struct Config {
        int grid_size;
        float grid_cell_length;
        };
    Config config;

    public:
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
        }
        std::cout << config.grid_size << '\n';

        publisher_occ_grid_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>("custom_map", 10);
        subscription_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>
        ("scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanSubscriber::laserscan_callback, this, std::placeholders::_1));
        }

    private:
        void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_ptr) const
        {
        RCLCPP_INFO(this->get_logger(), "I heard at -180: '%f'", msg_ptr->ranges[0]);
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