//#pragma once
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

// Include files required for RANSAC
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h> //Used for getMinMax3D
#include <fstream>
#include <sstream>
#include <string>



// using std::placeholders::_1;

// clear && colcon build --packages-select custom_nav_stack_pkg --symlink-install && source install/local_setup.bash && ros2 run custom_nav_stack_pkg cpp_executable
// sudo chown -R dev /opt/ros/foxy/share/ #to remove the squiggle error in vscode where "dev" is the username
// sudo apt install pcl-tools to install pcl_viewer to visualize the cluster for debugging

class MinimalSubscriber : public rclcpp::Node
{

  struct Config {
        int    num;
        std::string str;
        double flt;
        float leaf_size_x;
        float leaf_size_y;
        float leaf_size_z;
        float crop_min_x;
        float crop_min_y;
        float crop_min_z;
        float crop_max_x;
        float crop_max_y;
        float crop_max_z;
        int ransac_max_iterations;
        float ransac_distance_threshold;
        float cluster_tolerance;
        int cluster_min_size;
        int cluster_max_size;
      };
  Config config;
  
  public:
    
    
    MinimalSubscriber()    //Constructor which has the same name as that of class followed by a parentheses. A constructor in C++ is a special method that is automatically called when an object of a class is created. 
    : Node("minimal_subscriber")
    {
      std::cout << "Reached start of Public" << std::endl;
      
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
      
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>
      ("velodyne_points", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

      std::cout << "Reached end of Public" << std::endl;

      std::ifstream config_file_path("/home/dev/husky_ws/src/custom_nav_stack_pkg/src/config.txt");
      std::string line;
      while (std::getline(config_file_path, line)) {
          std::istringstream sin(line.substr(line.find("=") + 1));
          if (line.find("num") != -1)
              sin >> config.num;
          else if (line.find("str") != -1)
              sin >> config.str;
          else if (line.find("flt") != -1)
              sin >> config.flt;
          else if (line.find("leaf_size_x") != -1)
              sin >> config.leaf_size_x;
          else if (line.find("leaf_size_y") != -1)
              sin >> config.leaf_size_y;
          else if (line.find("leaf_size_z") != -1)
              sin >> config.leaf_size_z;
          else if (line.find("crop_min_x") != -1)
              sin >> config.crop_min_x;
          else if (line.find("crop_min_y") != -1)
              sin >> config.crop_min_y;
          else if (line.find("crop_min_z") != -1)
              sin >> config.crop_min_z;
          else if (line.find("ransac_max_iterations") != -1)
              sin >> config.ransac_max_iterations;
          else if (line.find("ransac_distance_threshold") != -1)
              sin >> config.ransac_distance_threshold;
          else if (line.find("cluster_tolerance") != -1)
              sin >> config.cluster_tolerance;
          else if (line.find("cluster_min_size") != -1)
              sin >> config.cluster_min_size;
          else if (line.find("cluster_max_size") != -1)
              sin >> config.cluster_max_size;
      }
      std::cout << config.num << '\n';
      std::cout << config.leaf_size_x << '\n';
      std::cout << config.crop_min_x << '\n';
      
      
      
    }

  private:

    
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr) const
    {
      std::cout << "Config crop min x is: " << config.crop_min_x << std::endl;
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
      // voxelGrid.setLeafSize(0.1, 0.1, 0.1);
      voxelGrid.setLeafSize(config.leaf_size_x, config.leaf_size_y, config.leaf_size_x);
      // apply the filter to dereferenced cloudVoxel
      voxelGrid.filter(*cloud_filtered_ptr);   

      std::cerr << "PointCloud after voxel filtering: " << cloud_filtered_ptr->width * cloud_filtered_ptr->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered_ptr) << ")." << std::endl;

      // STEP 2: Cropping Using CropBox; Reference: https://answers.ros.org/question/318183/how-to-use-cropbox-on-pointcloud2/

      // define a cropbox
      pcl::CropBox<pcl::PCLPointCloud2> cropBox;
      cropBox.setInputCloud(cloud_filtered_ptr);
      Eigen::Vector4f min_pt (-5.0f, -5.0f, -10.0f, 1.0); //(minX, minY, minZ, 1.0) in meter
      Eigen::Vector4f max_pt (5.0f, 5.0f, 10.0f, 1.0); //(maxX, maxY, maxZ, 1.0) in meter
      // Eigen::Vector4f min_pt (config.crop_min_x, config.crop_min_y, config.crop_min_z, 1.0f); //(minX, minY, minZ, 1.0) in meter
      // Eigen::Vector4f max_pt (config.crop_max_x, config.crop_max_y, config.crop_max_z, 1.0f);
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

      // STEP 3(a): Segmentation Using RANSAC Algorithm; Reference: https://pointclouds.org/documentation/classpcl_1_1_s_a_c_segmentation.html#a7e9ad0f4cd31e45c2ff03da17d0c9bce, https://pcl.readthedocs.io/projects/tutorials/en/latest/planar_segmentation.html, https://github.com/RAS2015-GROUP5/ras_computer_vision/blob/master/milestone_1_object_detection/src/object_detection.cpp
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      // Create the segmentation object
      // pcl::SACSegmentation<pcl::PCLPointCloud2> seg;
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *cloud_xyz_ptr);  //https://stackoverflow.com/questions/69716482/convert-from-pclpointcloudpclpointxyz-to-pclpclpointcloud2-ros-melodic  (we can using <PointT> or PointXYZ in all function it seems)
      // cloud_xyz_ptr->header =cloud_filtered_ptr->header;
      // cloud_xyz_ptr->width =cloud_filtered_ptr->width;
      // cloud_xyz_ptr->height =cloud_filtered_ptr->height;
      // cloud_xyz_ptr->is_dense =cloud_filtered_ptr->is_dense;
      // cloud_xyz_ptr->points =cloud_filtered_ptr->data;

      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      // seg.setMaxIterations (50);
      // seg.setDistanceThreshold (0.01);
      seg.setMaxIterations (config.ransac_max_iterations);
      seg.setDistanceThreshold (config.ransac_distance_threshold);
      
      seg.setInputCloud (cloud_xyz_ptr);
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

      
      // STEP 3(b): Extracting ground/ non-ground points // https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/master/README.md
        // If we want ground points, use extract.setNegative (false);
        // If we want non-ground, obstacle points, use extract.setNegative (true);
      
      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      extract.setInputCloud (cloud_xyz_ptr);
      extract.setIndices (inliers);
      extract.setNegative (true);
      extract.filter (*cloud_xyz_ptr);

      std::cerr << "PointCloud after RANSAC plane extraction, ground points: " << cloud_xyz_ptr->width * cloud_xyz_ptr->height 
       << " data points (" << pcl::getFieldsList (*cloud_xyz_ptr) << ")." << std::endl;

      // Step 4: Clustering using K-D treel; Reference: https://link.springer.com/chapter/10.1007/978-981-16-6460-1_57, https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/master/README.md, https://pcl.readthedocs.io/en/latest/cluster_extraction.html

      // Create the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud_xyz_ptr);
    
      // create the extraction object for the clusters
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      // specify euclidean cluster parameters
      // ec.setClusterTolerance (0.2); // 2cm
      // ec.setMinClusterSize (50);
      // ec.setMaxClusterSize (25000);
      ec.setClusterTolerance (config.cluster_tolerance); // 2cm
      ec.setMinClusterSize (config.cluster_min_size);
      ec.setMaxClusterSize (config.cluster_max_size);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloud_xyz_ptr);
      // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
      ec.extract (cluster_indices);
      // std::cout << "Cluster indices: " << cluster_indices[0] << std::endl;

      pcl::PCDWriter writer;

      int j = 0;
      for (const auto& cluster : cluster_indices)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices) {
          cloud_cluster->push_back((*cloud_xyz_ptr)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;   
        cloud_cluster->header = cloud_xyz_ptr->header;  
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        Eigen::Vector4f min_pt_cluster; //(minX, minY, minZ, 1.0) in meter
        Eigen::Vector4f max_pt_cluster; //(maxX, maxY, maxZ, 1.0) in meter
        pcl::getMinMax3D(*cloud_cluster,min_pt_cluster, max_pt_cluster); //Ref: https://github.com/PointCloudLibrary/pcl/blob/master/examples/common/example_get_max_min_coordinates.cpp
        std::cout << "Max x: " << max_pt_cluster[0] << std::endl;
        std::cout << "Max y: " << max_pt_cluster[1] << std::endl;
        std::cout << "Max z: " << max_pt_cluster[2] << std::endl;
        std::cout << "Min x: " << min_pt_cluster[0] << std::endl;
        std::cout << "Min y: " << min_pt_cluster[1] << std::endl;
        std::cout << "Min z: " << min_pt_cluster[2] << std::endl;
        sensor_msgs::msg::PointCloud2 ros_processed_pcl2; //Declaring a pointer using new was working but gave deprecated warning
        pcl::toROSMsg(*cloud_cluster, ros_processed_pcl2);
        publisher_->publish(ros_processed_pcl2);

        // std::stringstream ss;
        // ss << "cloud_cluster_" << j << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
      }

      



      //Publish data back to ROS2 for visualization
      // sensor_msgs::msg::PointCloud2 ros_processed_pcl2; //Declaring a pointer using new was working but gave deprecated warning
      // pcl_conversions::fromPCL(*cloud_filtered_ptr, ros_processed_pcl2_ptr);
      //pcl_conversions::fromPCL(*cloud_xyz_ptr, ros_processed_pcl2)
      // pcl::toROSMsg(*cloud_cluster, ros_processed_pcl2);

      // RCLCPP_INFO(this->get_logger(), "Process ROS2 PCL2, width is: '%d'", ros_processed_pcl2.width); //
      // publisher_->publish(ros_processed_pcl2);
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