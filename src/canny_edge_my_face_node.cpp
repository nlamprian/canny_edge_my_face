#include "canny_edge_my_face/canny_edge_my_face.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "canny_edge_my_face");
  CannyEdgeMyFace processor;
  ros::spin();
  return 0;
}
