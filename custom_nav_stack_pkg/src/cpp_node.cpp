#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>


// using std::placeholders::_1;

// clear && colcon build --packages-select custom_nav_stack_pkg --symlink-install && source install/local_setup.bash && ros2 run custom_nav_stack_pkg cpp_executable
// sudo chown -R dev /opt/ros/foxy/share/ #to remove the squiggle error in vscode where "dev" is the username

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()    //Constructor which has the same name as that of class followed by a parentheses. A constructor in C++ is a special method that is automatically called when an object of a class is created. 
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>
      ("velodyne_points", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      // std::cout << "Hello";
      RCLCPP_INFO(this->get_logger(), "I received the message , height is: '%d'", msg->height); //
      // RCLCPP_INFO(this->get_logger(), "I received the message"); //

      // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
      // pcl::PointCloud<pcl::PointXYZ> cloud; //Working, Reference: https://ros-developer.com/2017/02/23/converting-pcl-point-cloud-to-ros-pcl-cloud-message-and-the-reverse/
      // pcl::fromROSMsg (*msg, cloud); //Working
      // RCLCPP_INFO(this->get_logger(), "I received the PCL message , height is: '%d'", cloud.height); //WORKING
      // pcl::PCLPointCloud2::Ptr cloud;      
      // pcl::fromROSMsg (*msg, cloud);
      pcl::PCLPointCloud2 cloud;
      pcl_conversions::toPCL(*msg, cloud);
      // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::fromPCLPointCloud2(*cloud,*temp_cloud);
      RCLCPP_INFO(this->get_logger(), "I received the PCL message , height is: '%d'", cloud.height);
      // cloud.points[:].x = 4;
      // std::cout << "Complete ROS cloud is: " << msg << std::endl;
      // std::cout << "Complete ROS cloud type is: " << typeid(msg).name() << std::endl;
      // std::cout << "Complete PCL cloud is: " << cloud << std::endl;      
      // std::cout << "Complete PCL cloud type is: " << typeid(cloud).name() << std::endl;
      // std::cout << "PCL cloud points type is: " << typeid(cloud.points).name() << std::endl;
      // std::cout << "PCL 0 cloud is: " << cloud.points[0] << std::endl;      
      // std::cout << "PCL 15000 cloud is: " << cloud.points[15000] << std::endl;      
      // std::cout << "PCL 25000 cloud is: " << cloud.points[25000] << std::endl;      

 
      // // loop through the array elements
      // for (size_t i = 0; i < n; i++) {
      //     std::cout << input[i] << ' ';
      // }
      //Reading xyz values from the point cloud
      // BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points)
      // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
      
      //... populate cloud
      // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
      // viewer.showCloud (cloud.makeShared());
      
      std::cerr << "PointCloud before filtering: " << cloud.width * cloud.height 
             << " data points (" << pcl::getFieldsList (cloud) << ")." << std::endl;

      
      // Declaring pointer for the PCL "cloud" 
      pcl::PCLPointCloud2::Ptr cloudPtr (&cloud);

      // define a new container for the data
      // pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

      // define a voxelgrid
      // pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
      // set input to cloud
      // voxelGrid.setInputCloud(cloudPtr);
      // set the leaf size (x, y, z)
      // voxelGrid.setLeafSize(0.1, 0.1, 0.1);
      // apply the filter to dereferenced cloudVoxel
      // voxelGrid.filter(*cloud_filtered);   

      // std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
      //  << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

      // delete cloudPtr;
      std::cin.ignore();
      // std::cerr << "Reached callback end" << std::endl;
      // cloudPtr = NULL;
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  std::cout << "Inside main function now" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  
  rclcpp::shutdown();
  return 0;
}