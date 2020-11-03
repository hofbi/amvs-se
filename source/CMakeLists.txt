add_subdirectory(data)
add_subdirectory(decoder)
add_subdirectory(encoder)
add_subdirectory(filter)
add_subdirectory(mux)
add_subdirectory(quality)
add_subdirectory(statistics)

# Executeables
add_executable(
  single_encoder_node src/single_encoder_node.cpp src/single_encoder.cpp
                      include/single_encoder.h)

target_include_directories(single_encoder_node PRIVATE include)
target_link_libraries(
  single_encoder_node PRIVATE data_callbacks mux filter filter_spatio_temporal
                              filter_channel statistics)
