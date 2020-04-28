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
find_package(macaroons REQUIRED)

find_library(MACAROONS_LIB macaroons)
find_path(MACAROONS_INCLUDE_DIR macaroons.hpp)

include_directories(include ${MACAROONS_INCLUDE_DIR})

add_library(comms_nodes SHARED
            src/talker_node.cpp
            src/listener_node.cpp)
# target_compile_definitions(comms_nodes
#   PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(comms_nodes rclcpp rclcpp_components std_msgs)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
# if(NOT WIN32)
#   ament_environment_hooks(
#     "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
#     )
# endif()

add_executable(macaroon_demo src/macaroon_demo.cpp)
target_link_libraries(macaroon_demo comms_nodes ${MACAROONS_LIB})
# target_include_directories(macaroon_test PUBLIC ${MACAROONS_INCLUDE_DIR})
ament_target_dependencies(macaroon_demo rclcpp class_loader)
#target_include_directories(macaroon_test PRIVATE include)

install(TARGETS
  comms_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  macaroon_demo
  DESTINATION lib/${PROJECT_NAME})

print_all_variables()

ament_package()