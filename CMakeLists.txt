cmake_minimum_required(VERSION 3.0)
project(single_encoder)

find_package(catkin REQUIRED COMPONENTS roscpp roslaunch std_msgs sensor_msgs
                                        cv_bridge)

catkin_package()

add_subdirectory(source)

add_subdirectory(test)
