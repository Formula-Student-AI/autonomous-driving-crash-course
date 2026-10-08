cmake_minimum_required(VERSION 3.5)
project(vehicle_models)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# Global flag to cause add_library() to create shared libraries if on.
# If set to true, this will cause all libraries to be built shared
# unless the library was explicitly added as a static library.
option(BUILD_SHARED_LIBS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(state_lib REQUIRED)
find_package(angles REQUIRED)

set(dependencies
  yaml-cpp
  rclcpp
  Eigen3
  state_lib
  angles
)

# build library
add_library(${PROJECT_NAME} src/ros.cpp src/steering_model.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(${PROJECT_NAME}
  ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# export library
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(${dependencies})

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

install(FILES models.txt
        DESTINATION share/${PROJECT_NAME}/models)

install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
