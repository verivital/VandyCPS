cmake_minimum_required(VERSION 2.8.3)
project(cwru_opencv_common)
find_package(OpenCV REQUIRED)

find_package(rostest REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  camera_info_manager
  roscpp
  sensor_msgs
  std_msgs
  tf
  roslint
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
SET(LIB_TYPE STATIC) 

roslint_cpp()

add_service_files(DIRECTORY srv  
FILES image_label.srv
  )

#
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
  sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES opencv_ui_lib circle_detection_lib block_detection_lib grab_cut_lib projective_lib opencv_3d_lib opencv_2d_lib opencv_local_lib opencv_rot_lib color_model_lib ellipse_modeling_lib
    CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
    DEPENDS OpenCV
)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(circle_detection_lib
    src/circle_detection.cpp
)

add_library(block_detection_lib
    src/block_detection.cpp
)

add_library(opencv_3d_lib
    src/opencv_geometry_3d.cpp
)

add_library(opencv_2d_lib
    src/opencv_geometry_2d.cpp
)

add_library(opencv_ui_lib
    src/opencv_ui.cpp
)

add_library(opencv_local_lib
    src/opencv_local.cpp
)

add_library(opencv_rot_lib
    src/cv_rotations.cpp
)

add_library(projective_lib
    src/projective_geometry.cpp
)

add_library(ellipse_modeling_lib
    src/ellipse_modeling.cpp
    src/ellipse_modeling_numerical.cpp
)


add_library(grab_cut_lib
    src/grab_cut.cpp
)

add_library(color_model_lib
    src/color_model.cpp
)

add_library(manual_labeling_server src/manual_labeling.cpp)

target_link_libraries(manual_labeling_server
        opencv_ui_lib
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES})

add_dependencies(manual_labeling_server cwru_opencv_common_gencpp)


target_link_libraries(opencv_3d_lib
   projective_lib
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
 )

target_link_libraries(projective_lib
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
 )


ADD_EXECUTABLE(manual_labeling node/manual_labeling_node.cpp)

TARGET_LINK_LIBRARIES(manual_labeling
    manual_labeling_server
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)



ADD_EXECUTABLE(ellipse_iteration_test test/test_ellipse.cpp)

#ADD_EXECUTABLE(catheter_playback_node src/catheter_playback_node.cpp)

#ADD_EXECUTABLE(vesselness_image_filter_gpu_node src/gpu_src/vesselness_image_filter_gpu_node.cpp)

#ADD_EXECUTABLE(vesselness_image_filter_cpu src/cpu_src/vesselness_image_filter_gpu.cpp)
#ADD_DEPENDENCIES(beginner_tutorials_node T)

TARGET_LINK_LIBRARIES(ellipse_iteration_test
    ellipse_modeling_lib
    ${catkin_LIBRARIES}
)

TARGET_LINK_LIBRARIES(ellipse_modeling_lib
    projective_lib
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

target_link_libraries(block_detection_lib
    opencv_local_lib
    ${catkin_LIBRARIES}
)






# Add gtests


catkin_add_gtest(gtest_cv_rotations test/gtest_cv_rotations.cpp)

target_link_libraries(gtest_cv_rotations
        opencv_rot_lib
        ${OpenCV_LIBRERIES}
        ${catkin_LIBRARIES})



catkin_add_gtest(gtest_ellipse_matching test/gtest_ellipse_matching.cpp)

target_link_libraries(gtest_ellipse_matching
        ellipse_modeling_lib
        opencv_rot_lib
        ${OpenCV_LIBRERIES}
        ${catkin_LIBRARIES})


# gtesting for the projective geometry library:

catkin_add_gtest(gtest_point_array_projection test/gtest_point_array_projection.cpp)


target_link_libraries(gtest_point_array_projection
       projective_lib
       ${OpenCV_LIBRERIES}
       ${catkin_LIBRARIES})


#Add rostesting
        
#add_rostest_gtest(rostest_point_array_projection test/gtest_point_array_projection.test test/gtest_point_array_projection.cpp)

#target_link_libraries(rostest_point_array_projection
#      projective_lib
#      ${OpenCV_LIBRERIES}
#      ${catkin_LIBRARIES})
