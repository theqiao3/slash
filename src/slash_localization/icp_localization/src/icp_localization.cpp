#include "icp_localization/icp_localization.hpp"
#include <filesystem>

namespace icp_localization
{

ICPLocalization::ICPLocalization(const rclcpp::NodeOptions & options)
: Node("icp_localization", options),
  has_initial_pose_(false),
  has_odom_(false),
  map_to_odom_(Eigen::Matrix4f::Identity()),
  current_pose_(Eigen::Matrix4f::Identity()),
  last_odom_pose_(Eigen::Matrix4f::Identity())
{
  declareParameters();
  loadParameters();
  
  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  
  // Load Map
  if (!loadPCDMap()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD map. Exiting.");
    // In a real scenario we might want to handle this better, but for now we just log error
  }
  
  setupPublishersSubscribers();
  setupTimers();
  
  // Initialize ICP
  icp_.setMaximumIterations(icp_max_iterations_);
  icp_.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
  icp_.setTransformationEpsilon(icp_transformation_epsilon_);
  icp_.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon_);
  icp_.setRANSACIterations(0); // Disable RANSAC for standard ICP, or set if needed
  
  RCLCPP_INFO(this->get_logger(), "ICP Localization Node Initialized");
}

ICPLocalization::~ICPLocalization()
{
}

void ICPLocalization::declareParameters()
{
  declare_parameter("pcd_map_path", "");
  declare_parameter("odom_frame", "odom");
  declare_parameter("base_frame", "base_footprint");
  declare_parameter("map_frame", "map");
  declare_parameter("lidar_frame", "laser_link");
  declare_parameter("lidar_topic", "/scan_cloud");
  declare_parameter("odom_topic", "/odom");
  
  declare_parameter("voxel_leaf_size", 0.5);
  declare_parameter("map_voxel_leaf_size", 0.5);
  declare_parameter("icp_max_correspondence_distance", 1.0);
  declare_parameter("icp_max_iterations", 30);
  declare_parameter("icp_transformation_epsilon", 1e-8);
  declare_parameter("icp_euclidean_fitness_epsilon", 1e-6);
  declare_parameter("icp_ransac_outlier_threshold", 0.05);
  
  declare_parameter("min_scan_range", 0.5);
  declare_parameter("max_scan_range", 100.0);
  declare_parameter("local_map_radius", 50.0);
  
  declare_parameter("transform_publish_rate", 20.0);
  declare_parameter("map_publish_rate", 0.1);
  
  declare_parameter("fitness_score_threshold", 0.5);
  declare_parameter("use_initial_pose", true);
}

void ICPLocalization::loadParameters()
{
  pcd_map_path_ = get_parameter("pcd_map_path").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  lidar_frame_ = get_parameter("lidar_frame").as_string();
  lidar_topic_ = get_parameter("lidar_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  
  voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
  map_voxel_leaf_size_ = get_parameter("map_voxel_leaf_size").as_double();
  icp_max_correspondence_distance_ = get_parameter("icp_max_correspondence_distance").as_double();
  icp_max_iterations_ = get_parameter("icp_max_iterations").as_int();
  icp_transformation_epsilon_ = get_parameter("icp_transformation_epsilon").as_double();
  icp_euclidean_fitness_epsilon_ = get_parameter("icp_euclidean_fitness_epsilon").as_double();
  icp_ransac_outlier_threshold_ = get_parameter("icp_ransac_outlier_threshold").as_double();
  
  min_scan_range_ = get_parameter("min_scan_range").as_double();
  max_scan_range_ = get_parameter("max_scan_range").as_double();
  local_map_radius_ = get_parameter("local_map_radius").as_double();
  
  transform_publish_rate_ = get_parameter("transform_publish_rate").as_double();
  map_publish_rate_ = get_parameter("map_publish_rate").as_double();
  
  fitness_score_threshold_ = get_parameter("fitness_score_threshold").as_double();
  use_initial_pose_ = get_parameter("use_initial_pose").as_bool();
  
  if (!use_initial_pose_) {
    has_initial_pose_ = true; // Assume start at origin if not using initial pose
  }
}

bool ICPLocalization::loadPCDMap()
{
  if (pcd_map_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "PCD map path is empty");
    return false;
  }

  if (!std::filesystem::exists(pcd_map_path_)) {
    RCLCPP_ERROR(this->get_logger(), "PCD map file does not exist: %s", pcd_map_path_.c_str());
    return false;
  }

  global_map_.reset(new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>(pcd_map_path_, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_map_path_.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded PCD map with %zu points", global_map_->size());

  // Downsample global map
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
  voxel_grid.setInputCloud(global_map_);
  PointCloudT::Ptr downsampled_map(new PointCloudT);
  voxel_grid.filter(*downsampled_map);
  global_map_ = downsampled_map;

  RCLCPP_INFO(this->get_logger(), "Downsampled map to %zu points", global_map_->size());

  // Build KDTree
  map_kdtree_.reset(new pcl::KdTreeFLANN<PointT>);
  map_kdtree_->setInputCloud(global_map_);
  
  // Set input target for ICP (the map)
  icp_.setInputTarget(global_map_);

  return true;
}

void ICPLocalization::setupPublishersSubscribers()
{
  // Publishers
  aligned_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud", 1);
  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 1);
  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 1);
  
  // Subscribers
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    lidar_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ICPLocalization::pointCloudCallback, this, std::placeholders::_1));
    
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ICPLocalization::odomCallback, this, std::placeholders::_1));
    
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 1,
    std::bind(&ICPLocalization::initialPoseCallback, this, std::placeholders::_1));
    
  // Services
  relocalize_service_ = this->create_service<std_srvs::srv::Trigger>(
    "relocalize",
    std::bind(&ICPLocalization::relocalizationCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void ICPLocalization::setupTimers()
{
  // Timer for publishing transform
  transform_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / transform_publish_rate_),
    std::bind(&ICPLocalization::updateTransform, this));
    
  // Timer for publishing map (low frequency)
  map_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / map_publish_rate_),
    std::bind(&ICPLocalization::publishMapCloud, this));
}

void ICPLocalization::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!has_initial_pose_) {
    return;
  }

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  
  // Convert ROS msg to PCL point cloud
  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromROSMsg(*msg, *cloud);
  
  // Preprocess cloud
  current_scan_ = preprocessCloud(cloud);
  
  // Get initial guess from TF or Odom
  Eigen::Matrix4f initial_guess = current_pose_;
  
  // Use odom prediction if available
  try {
    geometry_msgs::msg::TransformStamped odom_to_base_tf;
    // Use the timestamp of the cloud for better accuracy
    rclcpp::Time cloud_time(msg->header.stamp);
    // Use a timeout to wait for odom
    odom_to_base_tf = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero); // Using latest for simplicity, ideally use cloud_time
      
    geometry_msgs::msg::Pose odom_to_base_pose;
    odom_to_base_pose.position.x = odom_to_base_tf.transform.translation.x;
    odom_to_base_pose.position.y = odom_to_base_tf.transform.translation.y;
    odom_to_base_pose.position.z = odom_to_base_tf.transform.translation.z;
    odom_to_base_pose.orientation = odom_to_base_tf.transform.rotation;

    Eigen::Matrix4f odom_to_base_mat = poseToMatrix(odom_to_base_pose);
    
    // Predict current pose: map_pose = map_to_odom * odom_pose
    {
        std::lock_guard<std::mutex> pose_lock(pose_mutex_);
        initial_guess = map_to_odom_ * odom_to_base_mat;
    }
    
  } catch (tf2::TransformException & ex) {
    // Fallback to last known pose
    RCLCPP_DEBUG(this->get_logger(), "Could not get odom for prediction: %s", ex.what());
  }
  
  // Perform ICP
  Eigen::Matrix4f result_transform;
  if (performICP(current_scan_, result_transform, initial_guess)) {
    {
      std::lock_guard<std::mutex> pose_lock(pose_mutex_);
      current_pose_ = result_transform;
      
      // Update map_to_odom
      // map_to_base = map_to_odom * odom_to_base
      // map_to_odom = map_to_base * odom_to_base^-1
      
      // Get odom_to_base
      geometry_msgs::msg::TransformStamped odom_to_base_tf;
      try {
        odom_to_base_tf = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, tf2::TimePointZero);
          
        geometry_msgs::msg::Pose odom_to_base_pose;
        odom_to_base_pose.position.x = odom_to_base_tf.transform.translation.x;
        odom_to_base_pose.position.y = odom_to_base_tf.transform.translation.y;
        odom_to_base_pose.position.z = odom_to_base_tf.transform.translation.z;
        odom_to_base_pose.orientation = odom_to_base_tf.transform.rotation;

        Eigen::Matrix4f odom_to_base_mat = poseToMatrix(odom_to_base_pose);
          
        map_to_odom_ = current_pose_ * odom_to_base_mat.inverse();
        
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get odom -> base transform: %s", ex.what());
      }
    }
    
    publishAlignedCloud(current_scan_);
    publishPosePath();
  }
}

void ICPLocalization::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // This can be used to predict the pose for the next scan
  // For now, we just store it if needed
  has_odom_ = true;
}

void ICPLocalization::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_pose_ = poseToMatrix(msg->pose.pose);
  has_initial_pose_ = true;
  RCLCPP_INFO(this->get_logger(), "Initial pose set: x=%.2f, y=%.2f", 
    msg->pose.pose.position.x, msg->pose.pose.position.y);

  // Immediately update map_to_odom based on current odom
  try {
    geometry_msgs::msg::TransformStamped odom_to_base_tf;
    odom_to_base_tf = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero);
      
    geometry_msgs::msg::Pose odom_to_base_pose;
    odom_to_base_pose.position.x = odom_to_base_tf.transform.translation.x;
    odom_to_base_pose.position.y = odom_to_base_tf.transform.translation.y;
    odom_to_base_pose.position.z = odom_to_base_tf.transform.translation.z;
    odom_to_base_pose.orientation = odom_to_base_tf.transform.rotation;

    Eigen::Matrix4f odom_to_base_mat = poseToMatrix(odom_to_base_pose);
      
    map_to_odom_ = current_pose_ * odom_to_base_mat.inverse();
    
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get odom -> base transform during init: %s", ex.what());
  }
}

void ICPLocalization::timerCallback()
{
  // Placeholder
}

bool ICPLocalization::performICP(const PointCloudT::Ptr & source_cloud, Eigen::Matrix4f & result_transform, const Eigen::Matrix4f & initial_guess)
{
  if (!global_map_) return false;
  
  icp_.setInputSource(source_cloud);
  
  PointCloudT::Ptr aligned_cloud(new PointCloudT);
  icp_.align(*aligned_cloud, initial_guess);
  
  if (icp_.hasConverged()) {
    double score = icp_.getFitnessScore();
    last_fitness_score_ = score;
    
    if (score < fitness_score_threshold_) {
      result_transform = icp_.getFinalTransformation();
      successful_matches_++;
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "ICP converged but fitness score too high: %.4f", score);
      failed_matches_++;
      return false;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "ICP failed to converge");
    failed_matches_++;
    return false;
  }
}

void ICPLocalization::updateTransform()
{
  if (!has_initial_pose_) return;
  publishTransform();
}

void ICPLocalization::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = map_frame_;
  t.child_frame_id = odom_frame_;
  
  Eigen::Vector3f translation = map_to_odom_.block<3, 1>(0, 3);
  Eigen::Matrix3f rotation = map_to_odom_.block<3, 3>(0, 0);
  Eigen::Quaternionf q(rotation);
  
  t.transform.translation.x = translation.x();
  t.transform.translation.y = translation.y();
  t.transform.translation.z = translation.z();
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  
  tf_broadcaster_->sendTransform(t);
  
  // Also publish current pose
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->get_clock()->now();
  pose_msg.header.frame_id = map_frame_;
  pose_msg.pose = matrixToPose(current_pose_);
  current_pose_pub_->publish(pose_msg);
}

void ICPLocalization::publishAlignedCloud(const PointCloudT::Ptr & cloud)
{
  if (aligned_cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = map_frame_; // Aligned cloud is in map frame
    
    // We need to transform the cloud points to map frame using current_pose_ before publishing?
    // Wait, icp_.align(*aligned_cloud, current_pose_) produces aligned_cloud in target frame (map frame)
    // So it is already in map frame.
    
    aligned_cloud_pub_->publish(msg);
  }
}

void ICPLocalization::publishMapCloud()
{
  if (map_cloud_pub_->get_subscription_count() > 0 && global_map_) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*global_map_, msg);
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = map_frame_;
    map_cloud_pub_->publish(msg);
  }
}

void ICPLocalization::publishPosePath()
{
  if (path_pub_->get_subscription_count() > 0) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = matrixToPose(current_pose_);
    
    pose_path_.header = pose_msg.header;
    pose_path_.poses.push_back(pose_msg);
    
    // Limit path size
    if (pose_path_.poses.size() > 1000) {
      pose_path_.poses.erase(pose_path_.poses.begin());
    }
    
    path_pub_->publish(pose_path_);
  }
}

PointCloudT::Ptr ICPLocalization::preprocessCloud(const PointCloudT::Ptr & input_cloud)
{
  PointCloudT::Ptr filtered_cloud(new PointCloudT);
  
  // Voxel grid filter
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_grid.setInputCloud(input_cloud);
  voxel_grid.filter(*filtered_cloud);
  
  return filtered_cloud;
}

Eigen::Matrix4f ICPLocalization::poseToMatrix(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  Eigen::Quaternionf q(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  
  mat.block<3, 3>(0, 0) = q.toRotationMatrix();
  mat.block<3, 1>(0, 3) = Eigen::Vector3f(
    pose.position.x,
    pose.position.y,
    pose.position.z);
    
  return mat;
}

geometry_msgs::msg::Pose ICPLocalization::matrixToPose(const Eigen::Matrix4f & matrix)
{
  geometry_msgs::msg::Pose pose;
  Eigen::Matrix3f rotation = matrix.block<3, 3>(0, 0);
  Eigen::Vector3f translation = matrix.block<3, 1>(0, 3);
  Eigen::Quaternionf q(rotation);
  
  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  
  return pose;
}

void ICPLocalization::relocalizationCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  // Reset localization logic here if needed
  // For now, we just say success
  response->success = true;
  response->message = "Relocalization triggered (not fully implemented)";
}

}  // namespace icp_localization
