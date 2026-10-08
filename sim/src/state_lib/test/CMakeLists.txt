find_package(eufs_gmock_matchers REQUIRED)

ament_add_gmock(test_state_vector test_state_vector.cpp)
target_link_libraries(test_state_vector
  state_lib
  eufs_gmock_matchers::eufs_gmock_matchers
  Eigen3::Eigen
)

ament_add_gmock(test_state_matrix test_state_matrix.cpp)
target_link_libraries(test_state_matrix
  state_lib
  eufs_gmock_matchers::eufs_gmock_matchers
  Eigen3::Eigen
)
