cmake_minimum_required(VERSION 3.8)
project(state_lib)

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
find_package(Eigen3 REQUIRED)

# Create dependencies variable
set(dependencies
    Eigen3
)

add_library(${PROJECT_NAME} INTERFACE)
ament_target_dependencies(${PROJECT_NAME} INTERFACE ${dependencies})
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} INTERFACE
    Eigen3::Eigen)

# Export
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

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

ament_python_install_package("state")

pybind11_add_module(_common_type SHARED
    src/common_type/python/state_vector.cpp
    src/common_type/python/state_2d_vector.cpp
)

target_link_libraries(_common_type PUBLIC
    state_lib
    fmt
)

# Install cython modules as sub-modules of the project
install(
    TARGETS
    _common_type
    DESTINATION "${PYTHON_INSTALL_DIR}/state"
)

# Tests
if(BUILD_TESTING)
    # Gtest
    find_package(ament_cmake_gmock REQUIRED)
    add_subdirectory(test)
    
    find_package(ament_cmake_pytest REQUIRED)
    set(_pytest_tests
      test/test_state_vector.py
    )

    foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV AMENT_PREFIX_INDEX=${ament_index_build_path} PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    )
    endforeach()
endif()

ament_package()
