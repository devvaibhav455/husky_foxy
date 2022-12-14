/*
 * ompl_example_2d.hpp
 *
 *  Created on: April 6, 2020
 *      Author: Dominik Belter
 *	 Institute: Instute of Robotics and Machine Intelligence, Poznan University of Technology
 */

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>


#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>

namespace ompl_example_2d {

/*!
 * 2D planner example class
 */
class Planner2D
{
public:

    Planner2D();
    

    /*!
   * plan path
   */
    nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid& globalMap);

private:
    /// node handle
    // ros::NodeHandle& nodeHandle;

    /// problem dim
    int dim;

    /// max step length
    double maxStepLength;

    /// bounds for the x axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordXBound;

    /// bounds for the y axis
    std::shared_ptr<ompl::base::RealVectorBounds> coordYBound;

    /// start position
    std::shared_ptr<ompl::base::ScopedState<>> start;

    /// goal position
    std::shared_ptr<ompl::base::ScopedState<>> goal;

    /// search space
    std::shared_ptr<ompl::base::StateSpace> space;

    /// configure node
    void configure(void);

    /// extract path
    nav_msgs::msg::Path extractPath(ompl::base::ProblemDefinition* pdef);
};

} /* namespace */
