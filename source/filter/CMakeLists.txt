# Filter Parameter
add_library(filter_parameter include/parameter/filter_parameter.h
                             src/filter_parameter.cpp)

target_include_directories(
  filter_parameter
  PRIVATE include/parameter
  INTERFACE include/parameter)

# Filter Base
add_library(filter_base include/base/base_image_filter.h
                        src/base_image_filter.cpp)

target_include_directories(
  filter_base
  PRIVATE include/base ${catkin_INCLUDE_DIRS}
  INTERFACE include/base ${catkin_INCLUDE_DIRS})

# Filter Spatio-Temporal
add_library(
  filter_spatio_temporal
  include/spatio_temporal/temporal_filter.h src/temporal_filter.cpp
  include/spatio_temporal/spatial_filter.h src/spatial_filter.cpp)

target_include_directories(
  filter_spatio_temporal
  PRIVATE include/spatio_temporal
  INTERFACE include/spatio_temporal)

target_link_libraries(filter_spatio_temporal PUBLIC filter_base
                                                    ${catkin_LIBRARIES})

# Filter Channel
add_library(filter_channel include/channel/channel_filter.h
                           src/channel_filter.cpp)

target_include_directories(
  filter_channel
  PRIVATE include/channel
  INTERFACE include/channel)

target_link_libraries(filter_channel PUBLIC filter_base)

# Filter Blur
add_library(
  filter_blur
  include/blur/none_filter.h
  src/none_filter.cpp
  include/blur/blur_filter.h
  src/blur_filter.cpp
  include/blur/gaussian_blur_filter.h
  src/gaussian_blur_filter.cpp
  include/blur/median_blur_filter.h
  src/median_blur_filter.cpp)

target_include_directories(
  filter_blur
  PRIVATE include/blur
  INTERFACE include/blur)

target_link_libraries(filter_blur PUBLIC filter_base)

# Filter
add_library(filter include/base_image_filter_factory.h
                   src/base_image_filter_factory.cpp)

target_include_directories(
  filter
  PRIVATE include
  INTERFACE include)

target_link_libraries(
  filter
  PUBLIC filter_base filter_parameter
  PRIVATE filter_blur)

target_compile_features(filter PRIVATE cxx_std_14)
