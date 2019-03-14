#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>

#include <perception/face_detector.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

using namespace sensor_msgs;
using namespace message_filters;

namespace perception {

class Demor {
	public:
		FaceDetector faceDetector;
		sensor_msgs::CameraInfoConstPtr camera_info;

		ros::Publisher marker_pub;
		ros::Publisher face2d_pub;

		void callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth);
};

void Demor::callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth) {

  // convert sensor_msgs::Images to cv::Mats
  cv_bridge::CvImagePtr rgb_ptr;
  cv_bridge::CvImagePtr depth_ptr;

  try {
    rgb_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
    depth_ptr = cv_bridge::toCvCopy(depth); // 32FC1
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    ros::shutdown();
  }

  // run & time face detection
  cv::Point3d point3d;
	// ROS_INFO("before: face 3d point: x=%f, y=%f, z=%f", point3d.x, point3d.y, point3d.z);
  bool face_detected = faceDetector.detectFace(rgb_ptr->image, depth_ptr->image, &point3d);
	// ROS_INFO("after: face 3d point: x=%f, y=%f, z=%f", point3d.x, point3d.y, point3d.z);;

  face2d_pub.publish(rgb_ptr->toImageMsg());

  // publish an cube marker whose position is the 3d point of the center of the largest/closest detected face
  visualization_msgs::Marker face_marker;
  face_marker.ns = "face";
  face_marker.id = 0;
  face_marker.header.frame_id = camera_info->header.frame_id;
  face_marker.type = visualization_msgs::Marker::CUBE;

	if (face_detected) {
		ROS_INFO("face detected...");
		face_marker.pose.position.x = point3d.x;
	  face_marker.pose.position.y = point3d.y;
	  face_marker.pose.position.z = point3d.z;
	} else {
		ROS_INFO("NO face detected...");
	}
  face_marker.pose.orientation.w = 1; // the identity orientation;

  face_marker.scale.x = 0.1;
  face_marker.scale.y = 0.1;
  face_marker.scale.z = 0.03;
  face_marker.color.g = 1;
  face_marker.color.a = 1;
	
  marker_pub.publish(face_marker);
}

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "face_detection");
  ros::NodeHandle nh;

  perception::Demor demor;

  // fetch CameraInfo
  demor.camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/head_camera/rgb/camera_info");

  ROS_INFO("received camear_info...");

  // setup the faceDetector;
  demor.faceDetector.set_cam_model(demor.camera_info);

  // setup published topics
  demor.marker_pub = nh.advertise<visualization_msgs::Marker>("face_marker", 1, true);
  demor.face2d_pub = nh.advertise<sensor_msgs::Image>("face_2d", 100);

  message_filters::Subscriber<Image> rgb_sub(nh, "/head_camera/rgb/image_raw", 1);
  message_filters::Subscriber<Image> depth_sub(nh, "/head_camera/depth_registered/image_raw", 1);

  typedef sync_policies::ApproximateTime<Image, Image> SyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
  Synchronizer<SyncPolicy> sync(SyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&perception::Demor::callback, &demor, _1, _2));

  ros::spin();

  return 0;
}
