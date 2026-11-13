cmake_minimum_required(VERSION 3.8)
project(bottle_grasp_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install the framework-independent modules without duplicating their sources.
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../grasp_pose_estimator.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../bottle_grasp_pipeline.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../yolo_segmentation.py"
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../grasp_visualization.py"
  DESTINATION "${PYTHON_INSTALL_DIR}")

install(PROGRAMS scripts/grasp_pose_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../best_train_multi.pt")
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../best_train_multi.pt"
    DESTINATION share/${PROJECT_NAME}/models)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../best_train_multi.rknn")
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../best_train_multi.rknn"
    DESTINATION share/${PROJECT_NAME}/models)
endif()

ament_package()
