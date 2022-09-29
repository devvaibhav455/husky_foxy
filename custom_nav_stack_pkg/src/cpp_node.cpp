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
// #include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>



// using std::placeholders::_1;

// clear && colcon build --packages-select custom_nav_stack_pkg --symlink-install && source install/local_setup.bash && ros2 run custom_nav_stack_pkg cpp_executable
// sudo chown -R dev /opt/ros/foxy/share/ #to remove the squiggle error in vscode where "dev" is the username

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()    //Constructor which has the same name as that of class followed by a parentheses. A constructor in C++ is a special method that is automatically called when an object of a class is created. 
    : Node("minimal_subscriber")
    {
      std::cout << "Reached start of Public" << std::endl;
      
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
      
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>
      ("velodyne_points", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

      std::cout << "Reached end og Public" << std::endl;

    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) const
    {
      // std::cout << "Hello";
      // RCLCPP_INFO(this->get_logger(), "I received the message , height is: '%d'", msg_ptr->height); //
      // RCLCPP_INFO(this->get_logger(), "I received the message"); //

      // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
      // pcl::PointCloud<pcl::PointXYZ> cloud; //Working, Reference: https://ros-developer.com/2017/02/23/converting-pcl-point-cloud-to-ros-pcl-cloud-message-and-the-reverse/
      // pcl::fromROSMsg (*msg, cloud); //Working
      // RCLCPP_INFO(this->get_logger(), "I received the PCL message , height is: '%d'", cloud.height); //WORKING
      // pcl::PCLPointCloud2::Ptr cloud;      
      // pcl::fromROSMsg (*msg, cloud);
      pcl::PCLPointCloud2::Ptr cloud_in_ptr(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(*msg_ptr, *cloud_in_ptr);
      // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // pcl::fromPCLPointCloud2(*cloud,*temp_cloud);
      // RCLCPP_INFO(this->get_logger(), "I received the PCL message , height is: '%d'", cloud.height);
      // RCLCPP_INFO_STREAM(get_logger(), "[Input PCL PointCloud] width " << cloud_in_ptr->width << " height " << cloud_in_ptr->height);
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
      

      
      std::cerr << "PointCloud before filtering: " << cloud_in_ptr->width * cloud_in_ptr->height 
             << " data points (" << pcl::getFieldsList (*cloud_in_ptr) << ")." << std::endl;

      
      // Declaring pointer for the PCL "cloud" 
      // pcl::PCLPointCloud2::Ptr cloudPtr (&cloud);

      // define a new container for the data
      pcl::PCLPointCloud2::Ptr cloud_filtered_ptr (new pcl::PCLPointCloud2 ());

      // STEP 1: Filtering Using VoxelGrid; Reference: https://codediversion.wordpress.com/2019/01/16/simple-ros-c-and-lidar-with-pcl-on-ubuntu-16-04/

      // define a voxelgrid
      pcl::VoxelGrid<pcl::PCLPointCloud2> voxelGrid;
      // set input to cloud
      voxelGrid.setInputCloud(cloud_in_ptr);
      // set the leaf size (x, y, z)
      voxelGrid.setLeafSize(0.1, 0.1, 0.1);
      // apply the filter to dereferenced cloudVoxel
      voxelGrid.filter(*cloud_filtered_ptr);   

      std::cerr << "PointCloud after voxel filtering: " << cloud_filtered_ptr->width * cloud_filtered_ptr->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered_ptr) << ")." << std::endl;

      // STEP 2: Cropping Using CropBox; Reference: https://answers.ros.org/question/318183/how-to-use-cropbox-on-pointcloud2/

      // define a cropbox
      pcl::CropBox<pcl::PCLPointCloud2> cropBox;
      cropBox.setInputCloud(cloud_filtered_ptr);
      Eigen::Vector4f min_pt (-5.0f, -5.0f, -10.0f, 1.0f); //(minX, minY, minZ, 1.0) in meter
      Eigen::Vector4f max_pt (5.0f, 5.0f, 10.0f, 1.0f); //(maxX, maxY, maxZ, 1.0) in meter
      // Cropbox slighlty bigger then bounding box of points
      cropBox.setMin (min_pt);
      cropBox.setMax (max_pt);
      // Indices
      std::vector<int> indices;
      cropBox.filter (indices);
      // Cloud
      cropBox.filter(*cloud_filtered_ptr);
      std::cerr << "PointCloud after cropbox filtering: " << cloud_filtered_ptr->width * cloud_filtered_ptr->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered_ptr) << ")." << std::endl;

      // STEP 3: Segmentation Using RANSAC Algorithm; Reference: https://pointclouds.org/documentation/classpcl_1_1_s_a_c_segmentation.html#a7e9ad0f4cd31e45c2ff03da17d0c9bce, https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html, https://github.com/RAS2015-GROUP5/ras_computer_vision/blob/master/milestone_1_object_detection/src/object_detection.cpp
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PCLPointCloud2> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);
      seg.setMaxIterations (50);
      
      seg.setInputCloud (cloud_filtered_ptr);
      seg.segment (*inliers, *coefficients);

      if (inliers->indices.size () == 0)
      {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        //Need to skip ransac part here
      }

      std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                          << coefficients->values[1] << " "
                                          << coefficients->values[2] << " " 
                                          << coefficients->values[3] << std::endl;

      std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;


      //Publish data back to ROS2 for visualization
      sensor_msgs::msg::PointCloud2 ros_processed_pcl2_ptr; //Declaring a pointer using new was working but gave deprecated warning
      pcl_conversions::fromPCL(*cloud_filtered_ptr, ros_processed_pcl2_ptr);
      RCLCPP_INFO(this->get_logger(), "Process ROS2 PCL2, width is: '%d'", ros_processed_pcl2_ptr.width); //
      publisher_->publish(ros_processed_pcl2_ptr);
      std::cout << "Reached callback end in Private" << std::endl;
      // std::cin.ignore();
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  std::cout << "Inside main function now" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  
  rclcpp::shutdown();
  return 0;
}