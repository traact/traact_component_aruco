cmake_minimum_required(VERSION 3.16)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CONAN_TRAACT_RUN_ENV_ROOT}/cmake)

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

include(traact_default_library_setup)

add_library(${CONAN_PACKAGE_NAME} SHARED
        src/ArucoModule.h
        src/ArucoModule.cpp
        src/ArucoInput.cpp
        src/ArucoOutput.cpp
        src/ArucoDebugOutput.cpp
        src/ArucoFractalModule.cpp
        src/ArucoFractalModule.h
        src/ArucoFractalDebugOutput.cpp
        src/ArucoFractalDebugOutput.h
        src/ArucoFractalInput.cpp
        src/ArucoFractalPoseOutput.cpp
        src/ArucoFractalPosition2dListOutput.cpp
        src/ArucoFractalPosition3dListOutput.cpp src/ArucoFractalTracker.cpp)

conan_target_link_libraries(${CONAN_PACKAGE_NAME})

include(traact_default_library_final)



