# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_VERBOSE_MAKEFILE ON)
set (EXTRA_COMPONENT_DIRS "./components/micro_ros_espidf_component/.")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(QuteeV2)


