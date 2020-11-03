add_library(decoder include/video_decoder.h src/video_decoder.cpp
                    include/av_decoder.h src/av_decoder.cpp)

target_include_directories(
  decoder
  PRIVATE include
  PRIVATE ${catkin_INCLUDE_DIRS} SYSTEM
  INTERFACE include)

target_link_libraries(
  decoder
  PRIVATE ${catkin_LIBRARIES}
  PUBLIC avcodec avutil swscale)

target_compile_features(decoder PRIVATE cxx_std_14)
