#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include "ros2_rs_pcl/segplane/rspcl_segplane_component.hpp"

// #include "rspcl_segplane_component.hpp"
RspclSegPlaneComponent::RspclSegPlaneComponent() : Node("pclsub_segplane")
{
  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/camera/depth/color/points", 
    10, 
    std::bind(&RspclSegPlaneComponent::timer_callback, this, std::placeholders::_1)\
  );

  using namespace std::chrono_literals;
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_plane", 10);
}

// #include "rspcl_segplane_component.hpp"
void RspclSegPlaneComponent::timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  pcl::PointCloud<PointNT>::Ptr cloud(new pcl::PointCloud<PointNT>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",cloud_msg->height,cloud_msg->width);

  // main prosessing //
  // define a new container for the data
  pcl::PointCloud<PointNT>::Ptr cloud_filtered(new pcl::PointCloud<PointNT>);

  // PassThrough Filter
  pcl::PassThrough<PointNT> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");  // x axis
  // extract point cloud between 1.0 and 1.5 m
  pass.setFilterLimits(0.0,1.0);
  // pass.setFilterLimitsNegative (true);   // extract range reverse
  pass.filter(*cloud_filtered);

  // Voxel Grid: pattern 1
  pcl::VoxelGrid<PointNT> voxelGrid;
  voxelGrid.setInputCloud(cloud_filtered);
  leaf_size_ = 0.005f;
  // set the leaf size (x, y, z)
  voxelGrid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  // apply the filter to dereferenced cloudVoxel
  voxelGrid.filter(*cloud_filtered);

  // Statistical Outlier Removal
  // pcl::StatisticalOutlierRemoval<PointNT> sor;
  // sor.setInputCloud(cloud_filtered);
  // sor.setMeanK(50);
  // sor.setStddevMulThresh(0.1);
  // sor.setNegative(false);
  // sor.filter (*cloud_filtered);

  // seg plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<PointNT> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.003);
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  for (size_t i = 0; i < inliers->indices.size (); ++i) {
    cloud_filtered->points[inliers->indices[i]].r = 0;  
    cloud_filtered->points[inliers->indices[i]].g = 200;  
    cloud_filtered->points[inliers->indices[i]].b = 0;  
  }  

  auto plane = extractByInliers(cloud_filtered, inliers, false);    // 平面

  sensor_msgs::msg::PointCloud2 sensor_msg;
  pcl::toROSMsg(*cloud_filtered, sensor_msg);
  publisher_->publish(sensor_msg);
}

pcl::PointCloud<PointNT>::Ptr RspclSegPlaneComponent::extractByInliers(
    const pcl::PointCloud<PointNT>::Ptr &input,
    const pcl::PointIndices::Ptr &indices,
    bool negative) // false: 平面だけ, true: 平面除外
{
    pcl::ExtractIndices<PointNT> extract;
    extract.setInputCloud(input);
    extract.setIndices(indices);
    extract.setNegative(negative);

    pcl::PointCloud<PointNT>::Ptr output(new pcl::PointCloud<PointNT>);
    extract.filter(*output);
    return output;
}