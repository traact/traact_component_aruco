cmake_minimum_required(VERSION 3.16)

set(TARGET_NAME traact_component_aruco)

project(${TARGET_NAME} VERSION "0.0.0" LANGUAGES CXX)

find_package(traact_base)
list(APPEND CMAKE_MODULE_PATH ${traact_base_INCLUDE_DIR})

include(traact_default_library_setup)

add_library(${TARGET_NAME} SHARED
        src/ArucoModule.h
        src/ArucoModule.cpp
        src/ArucoInput.cpp
        src/ArucoOutput.cpp
        src/ArucoDebugOutput.cpp
        src/ArucoFractalTracker.cpp)


find_package(traact_spatial REQUIRED)
find_package(traact_vision REQUIRED)
find_package(aruco REQUIRED)


target_link_libraries(${TARGET_NAME} PRIVATE traact_spatial::traact_spatial traact_vision::traact_vision opencv::opencv aruco::aruco)

include(traact_default_library_final)



