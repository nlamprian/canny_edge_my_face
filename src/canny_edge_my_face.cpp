#include <set>

#include <opencv2/imgproc.hpp>

#include "canny_edge_my_face/canny_edge_my_face.h"

CannyEdgeMyFace::CannyEdgeMyFace()
    : pnh_("~"),
      reconfigure_server_(pnh_),
      img_transport_(gnh_),
      canny_threshold_ratio_(3) {
  reconfigure_server_.setCallback(
      boost::bind(&CannyEdgeMyFace::reconfigureCallback, this, _1, _2));

  initializeDilation();

  img_publisher_ = img_transport_.advertise(pnh_.getNamespace() + "/image", 1);
  img_subscriber_ = img_transport_.subscribe(
      "image_raw", 1, &CannyEdgeMyFace::imageCallback, this);
}

CannyEdgeMyFace::~CannyEdgeMyFace() {}

void CannyEdgeMyFace::initializeDilation() {
  int dilation_size = 1;
  cv::Size dilation_kernel_size =
      cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1);
  cv::Point dilation_anchor(dilation_size, dilation_size);
  dilation_element_ = cv::getStructuringElement(
      cv::MORPH_RECT, dilation_kernel_size, dilation_anchor);
}

void CannyEdgeMyFace::processImage(cv::Mat& image) {
  cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  cv::blur(image, image, cv::Size(3, 3));
  cv::Canny(image, image, canny_low_threshold_,
            canny_low_threshold_ * canny_threshold_ratio_, canny_kernel_size_);
  cv::dilate(image, image, dilation_element_);
}

void CannyEdgeMyFace::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr img;
  try {
    img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception&) {
    ROS_ERROR_THROTTLE(1, "Could not convert from '%s' to 'bgr8'.",
                       msg->encoding.c_str());
    return;
  }

  processImage(img->image);
  img->encoding = "mono8";

  img_publisher_.publish(img->toImageMsg());
}

void CannyEdgeMyFace::reconfigureCallback(CannyEdgeMyFaceConfig& config,
                                          uint32_t /*level*/) {
  canny_low_threshold_ = config.canny_low_threshold;

  std::set<int> valid_kernel_sizes{3, 5, 7};
  if (valid_kernel_sizes.count(config.canny_kernel_size))
    canny_kernel_size_ = config.canny_kernel_size;
  else
    config.canny_kernel_size = canny_kernel_size_;
}
