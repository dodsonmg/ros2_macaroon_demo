cmake_minimum_required(VERSION 3.5)
project(macaroon_demo)

macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(macaroon_msgs REQUIRED)
find_package(libmacaroons REQUIRED)

add_executable(macaroon_demo
              src/macaroon_demo.cpp
              src/talker_node.cpp
              src/listener_node.cpp
              src/resource_base.cpp
              src/resource_owner.cpp
              src/resource_user.cpp)
target_include_directories(macaroon_demo PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(macaroon_demo rclcpp rclcpp_components std_msgs macaroon_msgs libmacaroons)

install(TARGETS
  macaroon_demo
  DESTINATION lib/${PROJECT_NAME})

add_executable(tofu_demo
  src/tofu_demo.cpp
  src/talker_node.cpp
  src/listener_node.cpp
  src/resource_base.cpp
  src/resource_owner.cpp
  src/resource_user.cpp)
target_include_directories(tofu_demo PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
ament_target_dependencies(tofu_demo rclcpp rclcpp_components std_msgs macaroon_msgs libmacaroons)

install(TARGETS
tofu_demo
DESTINATION lib/${PROJECT_NAME})

print_all_variables()

ament_package()