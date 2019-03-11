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
#include <time.h>
#include <visualization_msgs/Marker.h>

#include <perception/face_detector.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

using namespace sensor_msgs;
using namespace message_filters;

namespace perception {

/// \brief A simple hepler class for demo.
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
  // std::vector<Frame> lst;

  float start_tick = clock();

  cv::Point3d point3d;
  faceDetector.detectFace(rgb_ptr->image, depth_ptr->image, &point3d);
  float end_tick = clock();

  ROS_INFO("face detection runtime: %f", (end_tick - start_tick) / CLOCKS_PER_SEC);

  // if (!result) {
  //   ROS_INFO("no face detected...\n");
  //   return;
  // }

  // run & time FrameToCloud()
  // start_tick = clock();

  // std::vector<PointCloudC::Ptr> object_clouds;
  // PointCloudC::Ptr object_cloud;
  //
  // for (std::vector<Frame>::iterator it = lst.begin(); it != lst.end(); it++) {
  //   object_cloud = PointCloudC::Ptr(new PointCloudC());
  //
  //   if (faceDetector.FrameToCloud(rgb_ptr->image, depth_ptr->image, *it, object_cloud)) {
  //     object_clouds.push_back(object_cloud);
  //   }
  // }
  // alternatively, you can call Match to directly return a vector of object_clouds as well:
  // faceDetector.Match(rgb_ptr->image, depth_ptr->image, &object_clouds);
  // end_tick = clock();

  // ROS_INFO("%lu calls to FrameToCloud() total runtime: %f\n", lst.size(),  (end_tick - start_tick) / CLOCKS_PER_SEC);
  //
  // if (object_clouds.empty()) {
  //   ROS_INFO("no valid matched object coordinate...\n");
  //   return;
  // }
  //
  // // annotates rgb_ptr->image & publish it
  // ROS_INFO("#matched 2D objects: %lu\n", lst.size());
  // ROS_INFO("-----------");
  // for (int i = 0; i < lst.size(); i++) {
  //   Frame& f = lst[i];
  //   std::ostringstream stm;
  //   stm << i;
  //   rectangle( rgb_ptr->image, f.p1, f.p2, cv::Scalar(255, 255, 0), 4, 8, 0 );
  //   putText(rgb_ptr->image, stm.str(), cv::Point(f.p1.x - 10, f.p1.y - 10),
  //           cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(255, 255, 0), 2, CV_AA);
  //   ROS_INFO("frame score: %f, p1 pos: [%d, %d], index: %d", f.score, f.p1.x, f.p1.y, i);
  // }
  // ROS_INFO("-----------\n");
  //
  face2d_pub.publish(rgb_ptr->toImageMsg());
  //
  // // concatenate the vector of pointClouds into a single one & publish it
  // PointCloudC::Ptr pcl_cloud(new PointCloudC());
  // for (std::vector<PointCloudC::Ptr>::iterator it = object_clouds.begin(); it != object_clouds.end(); it++) {
  //   *pcl_cloud += **it;
  // }
  // ROS_INFO("#matched 3D objects: %lu", object_clouds.size());
  // ROS_INFO("pcl cloud size: %lu\n", pcl_cloud->size());
  //
  // sensor_msgs::PointCloud2 ros_cloud;
  // pcl::toROSMsg(*pcl_cloud, ros_cloud);
  // ros_cloud.header.frame_id = camera_info->header.frame_id; // head_camera_rgb_optical_frame
  //
  // cloud_pub.publish(ros_cloud);
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
