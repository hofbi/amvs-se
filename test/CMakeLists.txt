configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/akiyo-0.png
  ${CMAKE_CURRENT_BINARY_DIR}/quality/resources/akiyo-0.png COPYONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/akiyo-1.png
  ${CMAKE_CURRENT_BINARY_DIR}/quality/resources/akiyo-1.png COPYONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/mobile-0.png
  ${CMAKE_CURRENT_BINARY_DIR}/quality/resources/mobile-0.png COPYONLY)
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/resources/mobile-1.png
  ${CMAKE_CURRENT_BINARY_DIR}/quality/resources/mobile-1.png COPYONLY)

# Library unittests
catkin_add_gmock(
  ${PROJECT_NAME}_unittests
  main.cpp
  simple_test.cpp
  filter/none_filter_test.cpp
  filter/temporal_filter_test.cpp
  filter/spatial_filter_test.cpp
  filter/channel_filter_test.cpp
  filter/filter_parameter_test.cpp
  statistics/encoder_statistics_test.cpp
  statistics/statistics_collection_test.cpp
  mux/super_frame_mux_test.cpp
  mux/super_frame_demux_test.cpp
  mux/buffer_utils_test.cpp
  quality/vqm_state_test.cpp
  quality/mdvqm_test.cpp
  quality/stvqm_test.cpp
  quality/mda_test.cpp)

target_include_directories(${PROJECT_NAME}_unittests PRIVATE include)

target_link_libraries(
  ${PROJECT_NAME}_unittests
  filter
  filter_spatio_temporal
  filter_channel
  filter_blur
  statistics
  encoder
  mux
  decoder
  quality)

# ROS Tests

# Roslaunch Checks
roslaunch_add_file_check(${PROJECT_SOURCE_DIR}/launch DEPENDENCIES
                         single_encoder_node)
