# Copyright (c) Huawei Technologies Co., Ltd. 2019. All rights reserved.

# CMake lowest version requirement
cmake_minimum_required(VERSION 3.5.1)

# project information
project(openpose_det)

# Compile options
add_compile_options(-std=c++11)

# Specify target generation path
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../../outputs")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  "../../outputs")
set(CMAKE_INSTALL_PREFIX "../../../run")
set(CMAKE_OUTPUT_DIR "out")


set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

#find_package(Eigen3 REQUIRED)
#find_package(OpenCV REQUIRED)

# Header path
include_directories(
#${EIGEN3_INCLUDE_DIR}
.
$ENV{DDK_PATH}/include/inc/
$ENV{DDK_PATH}/include/third_party/protobuf/include
$ENV{DDK_PATH}/include/third_party/cereal/include
$ENV{DDK_PATH}/include/libc_sec/include
$ENV{HOME}/ascend_ddk/include
$ENV{DDK_PATH}/include/third_party/opencv/include

Common PracticalSocket DataInput ImagePreProcess SaveFilePostProcess MindInferenceEngine MindInferenceEngineGesture
)

if(type STREQUAL host)
    if(target STREQUAL "RC")
        add_compile_options(-DIS_RC)
    endif()

    # add host lib path
    link_directories($ENV{NPU_HOST_LIB})

    add_executable(main Main.cpp)
    add_library(Host SHARED Common/Common.cpp DataInput/DataInput.cpp SaveFilePostProcess/SaveFilePostProcess.cpp)


    if(target STREQUAL "RC")
        target_link_libraries(main matrixdaemon mmpa pthread dl rt )
        target_link_libraries(Host matrixdaemon hiai_common  opencv_world)
    else()
        target_link_libraries(main matrix mmpa pthread dl rt)
        target_link_libraries(Host matrix hiai_common opencv_world)
    endif()

    install(TARGETS main Host DESTINATION ${CMAKE_OUTPUT_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE)
else()
    add_compile_options(-DCPU_ONLY)

    # add device lib path
    link_directories($ENV{NPU_DEV_LIB})

    add_library(Device SHARED Common/Common.cpp PracticalSocket/PracticalSocket.cpp ImagePreProcess/ImagePreProcess.cpp MindInferenceEngine/MindInferenceEngine.cpp MindInferenceEngineGesture/MindInferenceEngineGesture.cpp)

    if(target STREQUAL "RC")
        target_link_libraries(Device Dvpp_api Dvpp_jpeg_decoder Dvpp_jpeg_encoder  opencv_world Dvpp_vpc idedaemon hiai_common)
    else()
        target_link_libraries(Device Dvpp_api Dvpp_jpeg_decoder Dvpp_jpeg_encoder Dvpp_vpc idedaemon)
    endif()

    install(TARGETS Device DESTINATION ${CMAKE_OUTPUT_DIR} PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE)
endif()
