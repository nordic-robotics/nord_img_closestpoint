cmake_minimum_required(VERSION 2.8.3)
project(nord_img_closestpoint)
set(CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
cv_bridge OpenCV  image_geometry std_msgs roscpp image_transport
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package()

add_executable(closestpoint src/test.cpp)
target_link_libraries(closestpoint ${catkin_LIBRARIES} ${OpenCV_LIBS})
