cmake_minimum_required(VERSION 3.8)
project(reeman_pose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(CURL REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(pozyx_to_reeman_pose
  "srv/PozyxToReeman.srv"
)

include_directories(${PROJECT_NAME}/include/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(reeman_pose_pub src/reeman_pose_pub.cpp src/httpClient.cpp)
#add_executable(pozyx_to_reeman_pose_srv src/pozyx_reeman_srv.cpp)
ament_target_dependencies(reeman_pose_pub rclcpp std_msgs geometry_msgs CURL)

#target_include_directories(pozyx_to_reeman_pose_srv PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

target_include_directories(reeman_pose_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  reeman_pose_pub
  DESTINATION lib/${PROJECT_NAME})

#install(TARGETS
#  pozyx_to_reeman_pose_srv 
#  DESTINATION lib/${PROJECT_NAME})

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
