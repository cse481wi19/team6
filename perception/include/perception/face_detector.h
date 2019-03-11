#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_geometry/pinhole_camera_model.h>

namespace perception {

class FaceDetector {

  public:
    FaceDetector();

    void set_cam_model(const sensor_msgs::CameraInfoConstPtr& camera_info);

    // perform 2d detection, return 3d face center points, and draw 2d bounding boxes on the input rgb
    void detectFace(cv::Mat& rgb, cv::Mat& depth, cv::Point3d* point3d);

    void drawFaces(cv::Mat& rgb, std::vector<cv::Rect> faces, cv::Point2d* p_center);

  private:
    image_geometry::PinholeCameraModel cam_model_;
    cv::CascadeClassifier face_cascade1;
    cv::CascadeClassifier face_cascade2;
    cv::CascadeClassifier face_cascade3;
    cv::CascadeClassifier face_cascade4;
};

}  // namespace perceptio
