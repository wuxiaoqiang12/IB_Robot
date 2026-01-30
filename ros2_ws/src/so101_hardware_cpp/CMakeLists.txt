cmake_minimum_required(VERSION 3.8)
project(so101_hardware_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nlohmann_json REQUIRED)

# Build Feetech SDK
add_library(ftservo_sdk STATIC
  FTServo_Linux/src/SCS.cpp
  FTServo_Linux/src/SCSCL.cpp
  FTServo_Linux/src/HLSCL.cpp
  FTServo_Linux/src/SMS_STS.cpp
  FTServo_Linux/src/SCSerial.cpp
)
set_target_properties(ftservo_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ftservo_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/FTServo_Linux/src>
  $<INSTALL_INTERFACE:include>
)

# Build hardware interface plugin
add_library(${PROJECT_NAME} SHARED
  src/so101_system_hardware.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  FTServo_Linux/src
)
target_link_libraries(${PROJECT_NAME} ftservo_sdk)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface so101_hardware_cpp_plugin.xml)

install(TARGETS ${PROJECT_NAME} ftservo_sdk
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
