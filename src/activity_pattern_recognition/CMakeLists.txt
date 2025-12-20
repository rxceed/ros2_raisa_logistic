cmake_minimum_required(VERSION 3.8)
project(activity_pattern_recognition)

# =========================================================
# Compiler
# =========================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# =========================================================
# ROS 2 dependencies
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


# =========================================================
# TensorFlow C API
# =========================================================

# Path where libtensorflow was installed
set(TENSORFLOW_C_INCLUDE_DIR
    "${CMAKE_CURRENT_LIST_DIR}/tensorflow/include"
)

set(TENSORFLOW_C_LIBRARY
    "${CMAKE_CURRENT_LIST_DIR}/tensorflow/lib/libtensorflow.so"
)

if(NOT EXISTS ${TENSORFLOW_C_LIBRARY})
  message(FATAL_ERROR "TensorFlow C library not found at ${TENSORFLOW_C_LIBRARY}")
endif()

# =========================================================
# Target
# =========================================================
add_executable(inference_pub
  src/activity_pattern_inference_pub.cpp
)

target_link_libraries(inference_pub
  ament_index_cpp::ament_index_cpp
)

target_include_directories(inference_pub PRIVATE
  include
  ${TENSORFLOW_C_INCLUDE_DIR}
)

target_link_libraries(inference_pub
  ${TENSORFLOW_C_LIBRARY}
)

ament_target_dependencies(inference_pub rclcpp std_msgs custom_interface geometry_msgs)

set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
set(CMAKE_BUILD_RPATH "$ORIGIN/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# ==========================================================
# INSTALL MODEL FILE
# ==========================================================

install(
  FILES ${CMAKE_CURRENT_LIST_DIR}/src/model/model.pb
  DESTINATION share/${PROJECT_NAME}/model
)



# =========================================================
# Install
# =========================================================
install(TARGETS inference_pub
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================================
# INSTALL TENSORFLOW C LIBRARIES
# ==========================================================

install(
  DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/tensorflow/lib/
  DESTINATION lib
  FILES_MATCHING
  PATTERN "*.so*"
)

install(DIRECTORY include/
  DESTINATION include/
)

# =========================================================
# Export
# =========================================================
ament_package()
