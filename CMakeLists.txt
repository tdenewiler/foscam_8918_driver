cmake_minimum_required(VERSION 2.8.3)
project(foscam_8918_driver)

find_package(catkin REQUIRED COMPONENTS camera_info_manager cv_bridge dynamic_reconfigure image_transport roscpp sensor_msgs)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/foscam_8918_driver.cfg)

catkin_package(CATKIN_DEPENDS camera_info_manager cv_bridge dynamic_reconfigure image_transport roscpp sensor_msgs)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/foscam_8918_driver.cpp src/nodes/foscam_8918_driver_node.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

install(PROGRAMS scripts/foscam_ptz DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
