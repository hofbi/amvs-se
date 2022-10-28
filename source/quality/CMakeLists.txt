find_package(OpenCV REQUIRED)

add_library(
  quality
  src/vqm_state.cpp
  include/vqm_state.h
  src/mdvqm.cpp
  include/mdvqm.h
  include/mdvqm_param.h
  src/stvqm.cpp
  include/stvqm.h
  include/stvqm_param.h
  src/mda.cpp
  include/mda.h)

target_include_directories(
  quality
  PRIVATE include ${OpenCV_INCLUDE_DIRS}
  INTERFACE include)

target_link_libraries(quality PRIVATE ${OpenCV_LIBRARIES})
