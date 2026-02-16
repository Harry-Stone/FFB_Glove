cmake_minimum_required(VERSION 3.8)
project(vibration_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(ALSA REQUIRED alsa)
pkg_check_modules(SNDFILE REQUIRED sndfile)

# 2. Define executable
add_executable(vibration_manager src/vibration_manager.cpp)

# 3. Headers
target_include_directories(vibration_manager PRIVATE 
  include
  ${ALSA_INCLUDE_DIRS}
  ${SNDFILE_INCLUDE_DIRS}
)

# 4. Link Everything (The "Modern" Way to avoid conflicts)
# We use PRIVATE for everything to be consistent
target_link_libraries(vibration_manager PRIVATE
  rclcpp::rclcpp
  ${std_msgs_LIBRARIES}
  ${ros_gz_interfaces_LIBRARIES}
  ${ALSA_LIBRARIES}
  ${SNDFILE_LIBRARIES}
)

# 5. Manual Include Hack (If headers still aren't found)
# Sometimes the variable above doesn't include the header path
target_include_directories(vibration_manager PRIVATE 
  ${ros_gz_interfaces_INCLUDE_DIRS}
)

# 6. Install
install(TARGETS vibration_manager
  DESTINATION lib/${PROJECT_NAME})

ament_package()