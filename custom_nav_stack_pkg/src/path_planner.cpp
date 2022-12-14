#include <memory>
#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
// using std::placeholders::_1;

#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "../include/ompl_example_2d/ompl_example_2d.hpp"

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>
#include <thread>

// clear && colcon build --packages-select custom_nav_stack_pkg --symlink-install && source install/local_setup.bash && ros2 run custom_nav_stack_pkg path_planner

// Ref: https://github.com/dominikbelter/ompl_example_2d , https://github.com/mlsdpk/ompl_2d_rviz_visualizer


nav_msgs::msg::OccupancyGrid globalMap;
int callback_done = 0;

class PathPlanner : public rclcpp::Node
{
  public:
    
    

    PathPlanner()
    : Node("path_planner")
    {
        std::cout << "Entered PathPlanner class" << std::endl;

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
        
        

        publisher_planned_path = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "custom_map", my_qos, std::bind(&PathPlanner::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr mapMsg) const
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: ");
      std::cout << "Entered occupancy grid callback function" << std::endl;
      std::cout << "Instantiating object planner_ from Planner2D class" << std::endl;
      ompl_example_2d::Planner2D planner_; //Planner2D class in ompl_example_2d.hpp | https://stackoverflow.com/questions/19615659/c-default-constructor-not-being-called
      std::cout << "Instantiated object planner_ from Planner2D class" << std::endl;
      
      nav_msgs::msg::Path plannedPath;
      plannedPath = planner_.planPath(*mapMsg); //Line 112 in ompl_example_2d.cpp
      
      publisher_planned_path->publish(plannedPath);
    }
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_planned_path;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  

  
  
  
  rclcpp::spin(std::make_shared<PathPlanner>());
  rclcpp::shutdown();
  return 0;
}