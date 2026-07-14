#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>


// ----点の型----
typedef pcl::PointXYZRGB PointNT;            // 座標 + 色 + 法線 + 曲率 はPointXYZRGBNormal
