find_package(Eigen3 REQUIRED)

ament_add_gmock(test_reader test_reader.cpp)
target_link_libraries(test_reader
  map_lib
)

ament_add_gmock(test_writer test_writer.cpp)
target_link_libraries(test_writer
  map_lib
)

ament_add_gmock(test_cone test_cone.cpp)
target_link_libraries(test_cone
  map_lib
  eufs_gmock_matchers::eufs_gmock_matchers
)

ament_add_gmock(test_map test_map.cpp)
target_link_libraries(test_map
  map_lib
  eufs_gmock_matchers::eufs_gmock_matchers
)

ament_add_gmock(test_negative_log_likelihood test_negative_log_likelihood.cpp)
target_link_libraries(test_negative_log_likelihood
  map_lib
)

ament_add_gmock(test_bhattacharyya test_bhattacharyya.cpp)
target_link_libraries(test_bhattacharyya
  map_lib
)

ament_add_gmock(test_association test_association.cpp)
target_link_libraries(test_association
  map_lib
)

ament_add_gmock(test_mahalanobis test_mahalanobis.cpp)
target_link_libraries(test_mahalanobis
  map_lib
)

ament_add_gmock(test_euclidean test_euclidean.cpp)
target_link_libraries(test_euclidean
  map_lib
)

ament_add_gmock(test_sequential_global test_sequential_global.cpp)
target_link_libraries(test_sequential_global
  map_lib
)

ament_add_gmock(test_sequential_random test_sequential_random.cpp)
target_link_libraries(test_sequential_random
  map_lib
)

ament_add_gmock(test_single test_single.cpp)
target_link_libraries(test_single
  map_lib
)

ament_add_gmock(test_thresholding test_thresholding.cpp)
target_link_libraries(test_thresholding
  map_lib
)

ament_add_gmock(test_fov test_fov.cpp)
target_link_libraries(test_fov
  map_lib
)

ament_add_gmock(test_transform test_transform.cpp)
target_link_libraries(test_transform
  map_lib
)

ament_add_gmock(test_align_svd test_align_svd.cpp)
target_link_libraries(test_align_svd
  map_lib
)

ament_add_gmock(test_cone_with_color_probability test_cone_with_color_probability.cpp)
target_link_libraries(test_cone_with_color_probability
  map_lib
)

install(FILES 
  test_map.csv
  empty_map.csv
  reference_map.csv
  DESTINATION share/${PROJECT_NAME}/test)
