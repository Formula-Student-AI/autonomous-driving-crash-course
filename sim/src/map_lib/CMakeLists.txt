cmake_minimum_required(VERSION 3.8)
project(map_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C99 and C++20
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(eufs_gmock_matchers REQUIRED)
find_package(eufs_logger REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)

set(map_lib_dependencies
  eufs_msgs
  Eigen3
  ament_cmake_gmock
  eufs_gmock_matchers
  eufs_logger
  geometry_msgs
  angles
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/reader.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${map_lib_dependencies})

# Check Ubuntu version to use libfmt or not
execute_process(
  COMMAND lsb_release -r -s
  OUTPUT_VARIABLE UBUNTU_VERSION)
if(${UBUNTU_VERSION} GREATER_EQUAL 22.04)
  set(LIBFMT "fmt::fmt")
  message(STATUS "Using libfmt.")
else()
  set(LIBFMT "")
  message(STATUS "Ignoring libfmt.")
endif()

target_link_libraries(${PROJECT_NAME}
  fmt::fmt
  eufs_gmock_matchers::eufs_gmock_matchers
  Eigen3::Eigen
  ${LIBFMT}
)

# Export
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Install library
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT  ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

find_package(ament_cmake_python REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/eufs_track_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES 
  plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pybind11_add_module(_type SHARED
    src/python/type/cone.cpp
    src/python/type/map.cpp
    src/python/type.cpp
)

target_link_libraries(_type PUBLIC
      map_lib
)

pybind11_add_module(_io SHARED
    src/python/io/reader.cpp
    src/python/io.cpp
)

target_link_libraries(_io PUBLIC
      map_lib
)

pybind11_add_module(_association SHARED
    src/python/association/single.cpp
    src/python/association/batch/sequential_global.cpp
    src/python/association/batch/sequential_random.cpp
    src/python/association/type/association.cpp
    src/python/association.cpp
)

target_link_libraries(_association PUBLIC
      map_lib
)

pybind11_add_module(_align SHARED
    src/python/association/align/align_svd.cpp
    src/python/align.cpp
)

target_link_libraries(_align PUBLIC
      map_lib
)

pybind11_add_module(_fov SHARED
    src/python/fov.cpp
)

target_link_libraries(_fov PUBLIC
      map_lib
)

# Install cython modules as sub-modules of the project
install(
      TARGETS
      _io
      _type
      _association
      _align
      _fov
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)


# Create dependencies variable
set(map_pub_dependencies
  rclcpp
  eufs_msgs
  jsoncpp
)

# Create node
add_executable(map_publisher
  src/map_publisher.cpp
)
ament_target_dependencies(map_publisher ${map_pub_dependencies})
target_include_directories(map_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(map_publisher
  map_lib
  jsoncpp
)

set(map_cap_dependencies
        rclcpp
        eufs_msgs
        eufs_logger
        )

# Create node
add_executable(map_capture
        src/map_capture.cpp
        )
ament_target_dependencies(map_capture ${map_cap_dependencies})
target_include_directories(map_capture PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(map_capture
        map_lib
        )

# Install executable
install(TARGETS map_publisher map_capture
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and maps
install(DIRECTORY
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${map_pub_dependencies} ${map_lib_dependencies})

# Tests
if(BUILD_TESTING)
  add_subdirectory(test)
  # find_package(ament_cmake_pytest REQUIRED)
  # set(_pytest_tests
  #   test/test_map.py
  #   test/test_align_svd.py
  #   test/test_single.py
  #   test/test_sequential_global.py
  #   test/test_sequential_random.py
  #   test/test_fov.py
  # )
  # foreach(_test_path ${_pytest_tests})
	#   get_filename_component(_test_name ${_test_path} NAME_WE)
	#   ament_add_pytest_test(${_test_name} ${_test_path}
  #     APPEND_ENV AMENT_PREFIX_INDEX=${ament_index_build_path} PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
  #     TIMEOUT 60
  #     WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  #  	)
  # endforeach()
endif()

ament_package()
