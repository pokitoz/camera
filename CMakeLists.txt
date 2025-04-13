cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS components)
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(camera_example)


# Open an IDF Terminal
# pip3 install catkin_pkg lark-parser colcon-common-extensions empy==3.3.4
