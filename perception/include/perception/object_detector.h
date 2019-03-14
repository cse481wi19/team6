#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ModelCoefficients.h"

#include "tf/transform_listener.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "shape_msgs/SolidPrimitive.h"
#include "visualization_msgs/Marker.h"

namespace perception {

class ObjectDetector {

  public:
    ObjectDetector(const ros::Publisher& marker_pub);

    void Callback(const sensor_msgs::PointCloud2& msg);

    bool cropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud);

    void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud);

    void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        pcl::PointIndices::Ptr indices,
                        pcl::ModelCoefficients::Ptr coeff);

    void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               pcl::PointIndices::Ptr surface_indices,
                               std::vector<pcl::PointIndices>* object_indices);

    bool checkShapeAndPose(shape_msgs::SolidPrimitive shape, geometry_msgs::Pose pose);

    void visualizeNewObjects(shape_msgs::SolidPrimitive shape, geometry_msgs::Pose obj_pose, size_t id,
                              visualization_msgs::Marker& object_marker, visualization_msgs::Marker& orient_marker);

    void deleteOldObjects(std::vector<visualization_msgs::Marker>& objects);

  private:
    ros::Publisher marker_pub_;
    tf::TransformListener tf_listener;
    std::vector<visualization_msgs::Marker> prev_objects;
};

}  // namespace perception
