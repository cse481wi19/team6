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

    // void publish3DFaceCenterPoint(const cv::Mat& rgb, const cv::Mat& depth, const cv::Point p);
    //
    void detect2DAndDisplay(cv::Mat& rgb);

    void drawFaces(cv::Mat& rgb, std::vector<cv::Rect> faces);

  private:
    image_geometry::PinholeCameraModel cam_model_;
    cv::CascadeClassifier face_cascade1;
    cv::CascadeClassifier face_cascade2;
    cv::CascadeClassifier face_cascade3;
    cv::CascadeClassifier face_cascade4;
};

}  // namespace perceptio
