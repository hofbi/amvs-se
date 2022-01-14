add_library(
  mux src/super_frame_mux.cpp include/super_frame_mux.h include/buffer_utils.h
      src/super_frame_demux.cpp include/super_frame_demux.h)

target_include_directories(
  mux
  PRIVATE include
  INTERFACE include)

target_link_libraries(mux PUBLIC encoder decoder quality)
