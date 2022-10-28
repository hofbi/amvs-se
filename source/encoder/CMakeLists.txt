add_library(encoder include/video_encoder.h src/video_encoder.cpp
                    include/x264_encoder.h src/x264_encoder.cpp)

target_include_directories(
  encoder
  PRIVATE include ${catkin_INCLUDE_DIRS}
  INTERFACE ${catkin_INCLUDE_DIRS} include)

target_link_libraries(
  encoder
  PUBLIC statistics x264 swscale avcodec avutil
  PRIVATE ${catkin_LIBRARIES})

target_compile_features(encoder PRIVATE cxx_std_14)
