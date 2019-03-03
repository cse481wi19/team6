#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "shape_msgs/SolidPrimitive.h"

namespace perception {

class ObjectDetector {

  public:
    ObjectDetector(const ros::Publisher& marker_pub);

    void Callback(const sensor_msgs::PointCloud2& msg);

    void cropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud);

    void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud);

    void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        pcl::PointIndices::Ptr indices,
                        pcl::ModelCoefficients::Ptr coeff);

    void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               pcl::PointIndices::Ptr surface_indices,
                               std::vector<pcl::PointIndices>* object_indices);

    bool checkShape(shape_msgs::SolidPrimitive shape);

  private:
    ros::Publisher marker_pub_;
};

}  // namespace perception
