#include <perception/face_detector.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>

#include <stdlib.h>
#include <assert.h>

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {

FaceDetector::FaceDetector() {
  face_cascade1.load("/home/team6/catkin_ws/src/cse481wi19/perception/face_features/haarcascade_frontalface_alt.xml");
  face_cascade2.load("/home/team6/catkin_ws/src/cse481wi19/perception/face_features/haarcascade_frontalface_alt2.xml");
  face_cascade3.load("/home/team6/catkin_ws/src/cse481wi19/perception/face_features/haarcascade_frontalface_alt_tree.xml");
  face_cascade4.load("/home/team6/catkin_ws/src/cse481wi19/perception/face_features/haarcascade_frontalface_default.xml");
}

void FaceDetector::set_cam_model(const sensor_msgs::CameraInfoConstPtr& camera_info) {
	cam_model_.fromCameraInfo(camera_info);
}

void FaceDetector::drawFaces(cv::Mat& rgb, std::vector<cv::Rect> faces, cv::Point2d* p_center) {

  double max_area = 0.0;

  for ( size_t i = 0; i < faces.size(); i++ ) {
    int left = faces[i].x;
    int top = faces[i].y;
    int right = left + faces[i].width;
    int bot = top + faces[i].height;

    double area = 1.0 * (right-left) * (bot-top);
    if (area > max_area) {
        max_area = area;
        p_center->x = (right-left) / 2;
        p_center->y = (bot-top) / 2;
    }

    Point tl(left, top);
    Point rb(right, bot);

    rectangle(rgb, tl, rb, Scalar(0, 255, 0));
  }
}

void FaceDetector::detectFace(cv::Mat& rgb, cv::Mat& depth, cv::Point3d* point3d) {
  Mat gray;
  cvtColor(rgb, gray, COLOR_BGR2GRAY);
  equalizeHist(gray, gray);

  //-- Detect faces
  std::vector<Rect> faces1;
  std::vector<Rect> faces2;
  std::vector<Rect> faces3;
  std::vector<Rect> faces4;

  face_cascade1.detectMultiScale(gray, faces1);
  face_cascade2.detectMultiScale(gray, faces2);
  face_cascade3.detectMultiScale(gray, faces3);
  face_cascade4.detectMultiScale(gray, faces4);

  // concatenate face vectors
  faces1.insert(faces1.end(), faces2.begin(), faces2.end());
  faces1.insert(faces1.end(), faces3.begin(), faces3.end());
  faces1.insert(faces1.end(), faces4.begin(), faces4.end());

  cv::Point2d p_center;
  drawFaces(rgb, faces1, &p_center);

  float dist = depth.at<float>(p_center.x, p_center.y);
  if (!isnan(dist)) {
    cv::Point3d p_3d = cam_model_.projectPixelTo3dRay(p_center);
    point3d->x = p_3d.x * dist;
    point3d->y = p_3d.y * dist;
    point3d->z = p_3d.z * dist;
  }
}

}
