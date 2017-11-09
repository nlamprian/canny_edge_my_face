#ifndef CANNY_EDGE_MY_FACE_H
#define CANNY_EDGE_MY_FACE_H

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "canny_edge_my_face/CannyEdgeMyFaceConfig.h"

class CannyEdgeMyFace {
  typedef canny_edge_my_face::CannyEdgeMyFaceConfig CannyEdgeMyFaceConfig;

 public:
  CannyEdgeMyFace();
  ~CannyEdgeMyFace();

 private:
  void initializeDilation();
  void processImage(cv::Mat& image);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void reconfigureCallback(CannyEdgeMyFaceConfig& config, uint32_t level);

  ros::NodeHandle gnh_, pnh_;

  dynamic_reconfigure::Server<CannyEdgeMyFaceConfig> reconfigure_server_;

  image_transport::ImageTransport img_transport_;
  image_transport::Subscriber img_subscriber_;
  image_transport::Publisher img_publisher_;

  const int canny_threshold_ratio_;
  int canny_low_threshold_;
  int canny_kernel_size_;
  cv::Mat dilation_element_;
};

#endif  // CANNY_EDGE_MY_FACE_H
