cmake_minimum_required(VERSION 2.8.3)
project(canny_edge_my_face)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	cv_camera
	dynamic_reconfigure
	roscpp
)

find_package(OpenCV 3.2 REQUIRED)

generate_dynamic_reconfigure_options(
	cfg/CannyEdgeMyFace.cfg
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS dynamic_reconfigure roscpp image_transport cv_bridge sensor_msgs
	DEPENDS OpenCV
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

file(GLOB HEADERS include/canny_edge_my_face/*.h)
file(GLOB SRCS src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SRCS} ${HEADERS})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
