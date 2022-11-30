#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
using std::placeholders::_1;

#include "ompl/geometric/planners/rrt/RRTstar.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "../include/ompl_example_2d/ompl_example_2d.hpp"

// clear && colcon build --packages-select custom_nav_stack_pkg --symlink-install && source install/local_setup.bash && ros2 run custom_nav_stack_pkg path_planner

// Ref: https://github.com/dominikbelter/ompl_example_2d , https://github.com/mlsdpk/ompl_2d_rviz_visualizer



class PathPlanner : public rclcpp::Node
{
  public:
    PathPlanner()
    : Node("path_planner")
    {
        std::cout << "Entered class" << std::endl;

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
        nav_msgs::msg::Path plannedPath;
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "custom_map", my_qos, std::bind(&PathPlanner::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: ");
    }
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlanner>());
  rclcpp::shutdown();
  return 0;
}