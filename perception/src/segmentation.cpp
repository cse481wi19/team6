#include "perception/segmentation.h"
#include "perception/typedefs.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/angles.h"
#include "pcl/common/common.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"


#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"


namespace perception {

// Computes the axis-aligned bounding box of a point cloud.
//
// Args:
//  cloud: The point cloud
//  pose: The output pose. Because this is axis-aligned, the orientation is just
//    the identity. The position refers to the center of the box.
//  dimensions: The output dimensions, in meters.
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
   Eigen::Vector4f min_pt, max_pt;
   pcl::getMinMax3D(*cloud, min_pt, max_pt);

   pose->position.x = (max_pt.x() + min_pt.x()) / 2;
   pose->position.y = (max_pt.y() + min_pt.y()) / 2;
   pose->position.z = (max_pt.z() + min_pt.z()) / 2;
   pose->orientation.w = 1; // the identity orientation

   dimensions->x = max_pt.x() - min_pt.x();
   dimensions->y = max_pt.y() - min_pt.y();
   dimensions->z = max_pt.z() - min_pt.z();
}

void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;

  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  pcl::ModelCoefficients coeff;
  seg.segment(indices_internal, coeff);

  // *indices = indices_internal;

  // Build custom indices that ignores points above the plane.
  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);
  
  for (size_t i = 0; i < cloud->size(); ++i) {
    const PointC& pt = cloud->points[i];
    float val = coeff.values[0] * pt.x + coeff.values[1] * pt.y +
                coeff.values[2] * pt.z + coeff.values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(i);
    }
  }

  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}

Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub)
    : surface_points_pub_(surface_points_pub), marker_pub_(marker_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);

  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  SegmentSurface(cloud, table_inliers);

  PointCloudC::Ptr table_cloud(new PointCloudC);

  // Extract subset of cloud into subset_cloud:
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.filter(*table_cloud);

  // publish table_cloud
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*table_cloud, msg_out);
  surface_points_pub_.publish(msg_out);

  // publish marker for the plane
  visualization_msgs::Marker table_marker;
  table_marker.ns = "table";
  table_marker.header.frame_id = "base_link";
  table_marker.type = visualization_msgs::Marker::CUBE;
  GetAxisAlignedBoundingBox(table_cloud, &table_marker.pose, &table_marker.scale);
  table_marker.color.r = 1;
  table_marker.color.a = 0.8;
  marker_pub_.publish(table_marker);
}

}  // namespace perception
