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
#include <bits/stdc++.h>
#include <rclcpp/qos.hpp>



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
// # This represents a 2-D grid map, in which each cell represents the probability of
// # occupancy.

// Header header 

// #MetaData for the map
// MapMetaData info

// # The map data, in row-major order, starting with (0,0).  Occupancy
// # probabilities are in the range [0,100].  Unknown is -1.
// int8[] data		


// clear && colcon build --packages-select custom_nav_stack_pkg --symlink-install && source install/local_setup.bash && ros2 run custom_nav_stack_pkg occupancy_grid_generator

std::vector<int> X;
std::vector<int> Y;
std::vector<float> dist_vlp;

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
            // std::cout << "Source: (" << x << ", " << y << ") ; Destination: (" << x2 << ", " << y2 << ")" << std::endl;
            X.clear(); Y.clear(); dist_vlp.clear();
            int eta = x;
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
                dist_vlp.push_back(sqrt((x-eta)*(x-eta) + (y-eta)*(y-eta))*config.grid_cell_length);
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
            

            static const rmw_qos_profile_t rmw_my_qos_profile_sensor_data =
            {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            1,
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_QOS_DEADLINE_DEFAULT,
            RMW_QOS_LIFESPAN_DEFAULT,
            RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
            false
            };

            auto my_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_my_qos_profile_sensor_data);
            // subscription_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>
            // ("scan", rclcpp::SensorDataQoS(), std::bind(&LaserScanSubscriber::laserscan_callback, this, std::placeholders::_1));


            subscription_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>
            ("scan", my_qos, std::bind(&LaserScanSubscriber::laserscan_callback, this, std::placeholders::_1));

        }

    private:
        void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg_ptr) const
        {
            // std::cout << "############## Callback function starts ##############" << std::endl;
            // RCLCPP_INFO(this->get_logger(), "I heard at 0 degree: '%f'", laserscan_msg_ptr->ranges[0]);
            // linev6(0,0,2,4);
            // linev6(4,2,0,0);
            // line(4,0,4,2);
            
            nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
            occupancy_grid_msg.header = laserscan_msg_ptr->header;
            occupancy_grid_msg.info.map_load_time.sec = 0;
            occupancy_grid_msg.info.map_load_time.nanosec = 0;
            occupancy_grid_msg.info.resolution = config.grid_cell_length; // The map resolution [m/cell]
            occupancy_grid_msg.info.width = config.grid_size; // Map width [cells]
            occupancy_grid_msg.info.height = config.grid_size; // Map height [cells]
            occupancy_grid_msg.info.origin.position.x = -(config.grid_size>>1)*config.grid_cell_length;
            occupancy_grid_msg.info.origin.position.y = (config.grid_size>>1)*config.grid_cell_length;
            occupancy_grid_msg.info.origin.position.z = 0.0;
            occupancy_grid_msg.info.origin.orientation.x = 0.0;
            occupancy_grid_msg.info.origin.orientation.y = 0.0;
            occupancy_grid_msg.info.origin.orientation.z = -0.7071068;
            occupancy_grid_msg.info.origin.orientation.w = 0.7071068;
            occupancy_grid_msg.data.reserve(config.grid_size*config.grid_size);
            std::vector<int8_t> vect1(config.grid_size*config.grid_size);
            fill(vect1.begin(), vect1.end(), -1);
            occupancy_grid_msg.data = vect1;

            std::vector<int> index_vect(config.grid_size*config.grid_size);
            index_vect.clear();
            // fill(index_vect.begin(), index_vect.end(), -1);
            


            int eta = (config.grid_size>>1);
            int x_dash;
            int y_dash;
            // std::cout << "No. of ranges in laserscan: " << laserscan_msg_ptr->ranges.size() << "Eta is: " << eta << std::endl;
            for(unsigned int i = 0; i < laserscan_msg_ptr->ranges.size() ; i++ ){
                    float angle_vlp = laserscan_msg_ptr->angle_min + i*(laserscan_msg_ptr->angle_increment);
                    float angle_vlp_degree =  angle_vlp*180/M_PI;
                    float m = tan(angle_vlp);
                    //Calculations to find the point of intersection of line with slope m passing through VLP sensor with grid's boundary
                    if ((angle_vlp_degree >= 0 && angle_vlp_degree <= 45) || (angle_vlp_degree <= 0 && angle_vlp_degree >= -45 )){
                        x_dash = 2*eta;
                        y_dash = round(-(m*eta) + eta);                      
                    }else if (angle_vlp_degree > 45 && angle_vlp_degree <= 135){
                        x_dash = round((eta/m) + eta);
                        y_dash = 0;                        
                    }else if ((angle_vlp_degree > 135 && angle_vlp_degree <= 180) || (angle_vlp_degree >= -180 && angle_vlp_degree <= -135)){
                        x_dash = 0;
                        y_dash = round((eta*m) + eta);
                    }else if (angle_vlp_degree > -135 && angle_vlp_degree < -45) {
                        x_dash = round(-(eta/m) + eta);
                        y_dash = 2*eta;
                    }
                    
                    line(eta, eta, x_dash, y_dash); // Calculating cells on matrix/ occupancy grid which constitute the line
                    
                    // if(i >= 2799 && i <= 2805){
                    //     std::cout << "i: " << i << " angle_vlp_degree: " << angle_vlp_degree << " slope/m: " << m << " eta: " << eta << " x_dash: " << x_dash << " y_dash: " << y_dash << std::endl;
                    //     std::cout << "X size: " << X.size() << " ; Y size: " << Y.size() << std::endl;
                    //     std::cout << "X is: "; 
                    //     for (auto i: X) { std::cout << i << " " ;} ;
                    //     std::cout << " ; Y is: "; 
                    //     for (auto i: Y) { std::cout << i << " " ;} std::cout << std::endl;
                    // }
                    
                    

                    if (laserscan_msg_ptr->ranges[i] != (laserscan_msg_ptr->range_max - 2)){
                        // std::cout << "\nRange is not 18.0, infact it is: " << laserscan_msg_ptr->ranges[i] << std::endl;
                        // std::cout << "Distance array is: ";
                        // for (auto k: dist_vlp) { std::cout << k << " " ;} ;
                        if (laserscan_msg_ptr->ranges[i] <= dist_vlp.back()){ //Calculate occupancy grid only if the range is within the required coverage of occupancy grid
                            auto it = std::find_if(std::begin(dist_vlp), std::end(dist_vlp), [&](float j){return j >= laserscan_msg_ptr->ranges[i]; });
                            int index_upto_obs = std::distance(dist_vlp.begin(), it); // Ref: https://www.techiedelight.com/find-index-element-vector-cpp/
                            // std::cout << " | Required Index is: " << index_upto_obs << std::endl;
                            // Set all cells in the laser direction upto index-1 as free(0); index as occupied(100) and beyond index as unknown(-1)
                            for (int j=0; j < index_upto_obs; j++){
                                occupancy_grid_msg.data.at(X[j]*config.grid_size + Y[j]) = 0;
                                index_vect.push_back(X[j]*config.grid_size + Y[j]);
                            }
                            occupancy_grid_msg.data.at(X[index_upto_obs]*config.grid_size + Y[index_upto_obs]) = 100;
                            index_vect.push_back(X[index_upto_obs]*config.grid_size + Y[index_upto_obs]);
                            for (unsigned int j=index_upto_obs; j < X.size(); j++){
                                occupancy_grid_msg.data.at(X[j]*config.grid_size + Y[j]) = -1;
                                index_vect.push_back(X[j]*config.grid_size + Y[j]);
                            }
                        }
                    }else if (laserscan_msg_ptr->ranges[i] == (laserscan_msg_ptr->range_max - 2)){
                       //It means that there's nothing obstructing that laser
                        for(unsigned int j = 0; j < X.size() ; j++ ){
                            // std::cout << "Index vector is: ";
                            // for (auto i: index_vect) { std::cout << i << " " ;} ;
                            // std::cout << X[j]*config.grid_size + Y[j] << std::endl;
                            // std::cout << vect1[0] << std::endl;
                            // std::cout << occupancy_grid_msg.data[0] << std::endl;
                        // occupancy_grid_msg.data[X[j]*config.grid_size + Y[j]] = 0;
                        // std::cout << "Setting (" << X[j] << "," << Y[j] << ") at index: " << X[j]*config.grid_size + Y[j] << " zero" << std::endl;
                        if( std::find(index_vect.begin(), index_vect.end(), X[j]*config.grid_size + Y[j]) == index_vect.end()){// Means that index is not found in index_vect. So, safe to set it to unoccupied/ free(0).
                            // std::cout << "Index not found in index_vect, so setting to zero j: " << j << " corresponding index: " << X[j]*config.grid_size + Y[j] << std::endl;
                            occupancy_grid_msg.data.at(X[j]*config.grid_size + Y[j]) = 0;
                        }
                        }
                    }
                   
                        
                        
                        
                        
                    
                    
                    

                    
                    // plt::plot(X,Y);
                    // plt::grid();
                    // plt::show();
                    // sleep(10);
            }

            // std:: cout << "Index zero: " << index_vect[0] << " Size: " << index_vect.size() << std::endl;
            // for (auto i: index_vect) { std::cout << i << " " ;} ;
            publisher_occ_grid_map->publish(occupancy_grid_msg);
            sleep(2);


            // std::cout << "\n############## Callback function ends ##############" << std::endl;
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