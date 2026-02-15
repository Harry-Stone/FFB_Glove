cmake_minimum_required(VERSION 3.20)
project(vibration_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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




find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)
pkg_check_modules(SNDFILE REQUIRED sndfile)

include_directories(
  ${ALSA_INCLUDE_DIRS}
  ${SNDFILE_INCLUDE_DIRS}
)

add_executable(vibration_manager src/vibration_manager.cpp)

target_link_libraries(vibration_manager PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${ALSA_LIBRARIES}
  ${SNDFILE_LIBRARIES}
)

ament_package()