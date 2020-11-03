add_library(data_callbacks src/image_callback.cpp include/image_callback.h)

target_include_directories(
  data_callbacks
  PRIVATE include SYSTEM
  INTERFACE include)

target_link_libraries(
  data_callbacks
  PUBLIC encoder decoder quality
  PRIVATE filter_base)
