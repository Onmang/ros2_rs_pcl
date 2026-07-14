#ifndef ROS2_RS_PCL__RSPCL_SEGPLANE_COMPONENT_HPP_
#define ROS2_RS_PCL__RSPCL_SEGPLANE_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rspcl_segplane_types.h"

class RspclSegPlaneComponent : public rclcpp::Node
{
  public:
    RspclSegPlaneComponent();

  private:
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    pcl::PointCloud<PointNT>::Ptr extractByInliers(
      const pcl::PointCloud<PointNT>::Ptr &input,
      const pcl::PointIndices::Ptr &indices,
      bool negative = false); // false: 平面だけ, true: 平面除外
    double leaf_size_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    size_t count_;
};

#endif // ROS2_RS_PCL__RSPCL_SEGPLANE_COMPONENT_HPP_