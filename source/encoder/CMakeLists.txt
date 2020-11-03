add_library(encoder include/video_encoder.h src/video_encoder.cpp
                    include/x264_encoder.h src/x264_encoder.cpp)

target_include_directories(
  encoder
  PRIVATE include
  PRIVATE ${catkin_INCLUDE_DIRS} SYSTEM
  INTERFACE ${catkin_INCLUDE_DIRS} SYSTEM
  INTERFACE include)

target_link_libraries(
  encoder
  PUBLIC statistics
  PUBLIC x264 swscale avcodec avutil
  PRIVATE ${catkin_LIBRARIES})

target_compile_features(encoder PRIVATE cxx_std_14)
