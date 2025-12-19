cmake_minimum_required(VERSION 3.8)
project(reeman_api_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(reeman_api_interface REQUIRED)
find_package(CURL REQUIRED)

add_executable(get_reeman_pose_server src/get_reeman_pose_srv.cpp src/httpClient.cpp)
ament_target_dependencies(get_reeman_pose_server rclcpp reeman_api_interface CURL)

add_executable(get_reeman_speed_server src/get_reeman_speed_srv.cpp src/httpClient.cpp)
ament_target_dependencies(get_reeman_speed_server rclcpp reeman_api_interface CURL)

add_executable(post_reeman_speed_server src/post_reeman_speed_srv.cpp src/httpClient.cpp)
ament_target_dependencies(post_reeman_speed_server rclcpp reeman_api_interface CURL)

add_executable(post_reeman_max_speed_server src/post_reeman_max_speed_srv.cpp src/httpClient.cpp)
ament_target_dependencies(post_reeman_max_speed_server rclcpp reeman_api_interface CURL)

add_executable(post_reeman_nav_server src/post_reeman_nav_srv.cpp src/httpClient.cpp)
ament_target_dependencies(post_reeman_nav_server rclcpp reeman_api_interface CURL)

add_executable(post_reeman_cancel_server src/post_reeman_cancel_srv.cpp src/httpClient.cpp)
ament_target_dependencies(post_reeman_cancel_server rclcpp reeman_api_interface CURL)

add_executable(post_reeman_nav_point_server src/post_reeman_nav_point_srv.cpp src/httpClient.cpp)
ament_target_dependencies(post_reeman_nav_point_server rclcpp reeman_api_interface CURL)


include_directories(${PROJECT_NAME}/include/${PROJECT_NAME})

target_include_directories(get_reeman_pose_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(get_reeman_speed_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(post_reeman_speed_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(post_reeman_max_speed_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(post_reeman_nav_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(post_reeman_cancel_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(post_reeman_nav_point_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  get_reeman_pose_server
  get_reeman_speed_server
  post_reeman_speed_server
  post_reeman_max_speed_server
  post_reeman_nav_server
  post_reeman_cancel_server
  post_reeman_nav_point_server
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
