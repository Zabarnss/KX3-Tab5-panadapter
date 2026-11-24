cmake_minimum_required(VERSION 3.20)

# ESP-IDF automatically discovers components under ./components. The
# ESP Component Manager resolves the Tab5 BSP, ESPP framework, and Espressif
# USB packages declared in main/idf_component.yml.
set(ENV{IDF_COMPONENT_MANAGER} "1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(KX3_Tab5_Panadapter)

set(CMAKE_CXX_STANDARD 23)
