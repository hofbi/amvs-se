add_library(
  statistics
  include/encoder_statistics.h src/encoder_statistics.cpp
  include/statistics_collection.h src/statistics_collection.cpp
  include/frame_stats.h)

target_include_directories(
  statistics
  PRIVATE include
  INTERFACE include)

target_link_libraries(statistics PUBLIC filter_parameter)
