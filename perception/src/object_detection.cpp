#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "perception/object_detector.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detection");
  ros::NodeHandle nh;

  // marker publisher for the segmented plane
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("object_markers", 1, true);

  perception::ObjectDetector detector(marker_pub);
  ros::Subscriber sub = nh.subscribe("cloud_in", 1, &perception::ObjectDetector::Callback, &detector);

  ros::spin();
  return 0;
}
