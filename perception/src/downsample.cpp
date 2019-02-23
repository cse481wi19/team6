#include "perception/downsample.h"
#include "perception/typedefs.h"

#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/common.h"

#include "ros/ros.h"

namespace perception {

Downsampler::Downsampler(const ros::Publisher& pub) : pub_(pub) {}

void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  PointCloudC::Ptr downsampled_cloud(new PointCloudC());
  pcl::VoxelGrid<PointC> vox;
  vox.setInputCloud(cloud);
  double voxel_size;
  ros::param::param("voxel_size", voxel_size, 0.01);
  vox.setLeafSize(voxel_size, voxel_size, voxel_size);
  vox.filter(*downsampled_cloud);
  ROS_INFO("Downsampled to %ld points", downsampled_cloud->size());

  // get min/max of downsampled clouds
  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*downsampled_cloud, min_pcl, max_pcl);
  ROS_INFO("min_x: %f, max_x: %f", min_pcl.x, max_pcl.x);

  // publish downsampled cloud
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*downsampled_cloud, msg_out);
  pub_.publish(msg_out);
}

}  // namespace perception
