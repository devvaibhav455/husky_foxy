#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
using std::placeholders::_1;

// colcon build --packages-select custom_nav_stack_pkg --symlink-install && source install/local_setup.bash && ros2 run custom_nav_stack_pkg cpp_executable

class MinimalSubscriber : public rclcpp::Node
{
  public:
     
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne_points", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      // std::cout << "Hello";
      RCLCPP_INFO(this->get_logger(), "I received the message , height is: '%d'", msg->height); //
      // RCLCPP_INFO(this->get_logger(), "I received the message"); //
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  std::cout << "Inside main function now";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}