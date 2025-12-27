/*
 * ImageToGridmapDemo.cpp
 *
 *  Created on: May 4, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <string>
#include <utility>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

#include "grid_map_demos/PCDToGridmapDemo.hpp"
namespace grid_map_demos
{

PCDToGridmapDemo::PCDToGridmapDemo()
: Node("pcd_to_gridmap_demo"),
  map_(grid_map::GridMap({"elevation"})),
  gridMapPclLoader(this->get_logger()),
  filterChain_("grid_map::GridMap")
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);

    readParameters();
    gridMapPublisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
    "grid_map", rclcpp::QoS(1).transient_local());
    updateTimer = rclcpp::create_timer(this,
                                    this->get_clock(),
                                    rclcpp::Duration::from_seconds(1),
                                    std::bind(&PCDToGridmapDemo::timerCallback, this));
    // Setup filter chain.
    if (filterChain_.configure(
        filterChainParametersName_, this->get_node_logging_interface(),
        this->get_node_parameters_interface()))
    {
        RCLCPP_INFO(this->get_logger(), "Filter chain configured.");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain!");
        rclcpp::shutdown();
        return;
    }
}

PCDToGridmapDemo::~PCDToGridmapDemo()
{
}

bool PCDToGridmapDemo::readParameters()
{
    this->declare_parameter("config_file_path", std::string());
    this->declare_parameter("filter_chain_parameter_name", std::string("filters"));
    this->declare_parameter("map_frame_id", std::string());
    this->declare_parameter("pcd_file_path", std::string());
    this->declare_parameter("point_min_z", -100.0);
    this->declare_parameter("point_max_z", 100.0);

    double point_min_z, point_max_z;
    this->get_parameter("config_file_path", configFilePath_);
    this->get_parameter("filter_chain_parameter_name", filterChainParametersName_);
    this->get_parameter("map_frame_id", mapFrameId_);
    this->get_parameter("pcd_file_path", PCDFilePath_);
    this->get_parameter("point_min_z", point_min_z);
    this->get_parameter("point_max_z", point_max_z);

    gridMapPclLoader.loadParameters(configFilePath_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(PCDFilePath_, *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Could not read PCD file: %s", PCDFilePath_.c_str());
        return false;
    }

    // Apply PassThrough filter for Z axis
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(point_min_z, point_max_z);
    pass.filter(*cloud_filtered);

    cloud_src = cloud_filtered;
    RCLCPP_INFO(this->get_logger(), "Loaded and filtered PCD with %zu points (original: %zu)", 
                cloud_src->size(), cloud->size());
    
    return true;
}

void PCDToGridmapDemo::timerCallback()
{
    gridMapPclLoader.setInputCloud(cloud_src);
    gridMapPclLoader.preProcessInputCloud();
    gridMapPclLoader.initializeGridMapGeometryFromInputCloud();
    gridMapPclLoader.addLayerFromInputCloud("elevation");
    map_ = gridMapPclLoader.getGridMap();
    map_.setFrameId(mapFrameId_);
    grid_map::GridMap outputMap;
    if (!filterChain_.update(map_, outputMap))
    {
        RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
        return;
    }
    auto message = grid_map::GridMapRosConverter::toMessage(outputMap);
    message->header.stamp = this->now();
    gridMapPublisher_->publish(std::move(message));
}

}  // namespace grid_map_demos
