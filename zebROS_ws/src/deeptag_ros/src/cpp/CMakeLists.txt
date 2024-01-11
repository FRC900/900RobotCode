cmake_minimum_required(VERSION 3.18)
set (CMAKE_BUILD_TYPE Debug)
set (CMAKE_EXPORT_COMPILE_COMMANDS True)
project(tensorrt_cpp_api)
enable_language(CUDA)

# Use ccache to speed up rebuilds
include(cmake/ccache.cmake)

# Set C++ version and optimization level
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG -Wno-deprecated-declarations")

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
# For finding FindTensorRT.cmake
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# include(cmake/CPM.cmake)

# # This will automatically clone CCCL from GitHub and make the exported cmake targets available
# CPMAddPackage(
#     NAME CCCL
#     GITHUB_REPOSITORY nvidia/cccl
#     GIT_TAG main # Fetches the latest commit on the main branch
# )

# Use the correct version of CUDA
set(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda)

# We require CUDA, OpenCV, and TensorRT
find_package(TensorRT REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_library(LIBNVTOOLSEXT nvToolsExt PATHS ${CMAKE_CUDA_IMPLICIT_LINK_DIRECTORIES})

add_library(tensorrt_cpp_api STATIC
    src/aruco_dict.cpp
    src/confidence_filter.cu
    src/cuda_event_timing.cpp
    src/cuda_ssd_preprocess.cu
    src/cuda_image_tile.cu
    src/cuda_utils.cpp
    src/decoder_engine.cpp
    src/decoder_preprocess.cu
    src/decoder_softmax.cu
    src/deeptag.cpp
    src/deeptag_impls.cpp
    src/detection_engine.cpp
    src/engine.cpp
    src/grid_prior.cu
    src/gpu_image_wrapper.cpp
    src/marker_dict.cpp
    src/pose_estimator.cpp
    src/roi_generator.cpp
    src/softmax.cu
    src/ssd_grid_prior.cu
    src/ssd_tag_keypoint.cu
    src/stag_decoder.cpp
    src/stag_detector.cpp
    src/stage1_grid.cu
    src/stage1_grid_group.cu
    src/stage1_ssd_group.cu
    src/stage2_corners.cu
    src/stage2_keypoint.cu
    src/stage2_keypoint_group.cu
    src/stage2_keypoint_trust.cu
    src/suppress_and_average_keypoints.cu
    src/unit_arucotag.cpp
    src/unit_chessboard_tag.cpp
    src/unit_tag_template.cpp
    src/warp_perspective_points.cpp
)

target_include_directories(tensorrt_cpp_api PUBLIC ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${TensorRT_INCLUDE_DIRS})
target_link_libraries(tensorrt_cpp_api PUBLIC ${OpenCV_LIBS} ${CUDA_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${TensorRT_LIBRARIES} ${LIBNVTOOLSEXT})
# target_link_libraries(tensorrt_cpp_api PRIVATE CCCL::CCCL)
set_target_properties(tensorrt_cpp_api PROPERTIES CUDA_ARCHITECTURES "86")
set_property(TARGET tensorrt_cpp_api PROPERTY CUDA_SEPARABLE_COMPILATION ON)

add_executable(run_inference_benchmark src/main.cpp)
target_link_libraries(run_inference_benchmark PUBLIC tensorrt_cpp_api)
# target_link_libraries(run_inference_benchmark PRIVATE CCCL::CCCL)
set_target_properties(run_inference_benchmark PROPERTIES CUDA_ARCHITECTURES "86")

add_executable(softmax_test src/softmax_test.cpp)
target_link_libraries(softmax_test PUBLIC tensorrt_cpp_api)
# target_link_libraries(softmax_test PRIVATE CCCL::CCCL)
set_target_properties(softmax_test PROPERTIES CUDA_ARCHITECTURES "86")

add_executable(fine_grid_test src/fine_grid_test.cpp)
target_link_libraries(fine_grid_test PUBLIC tensorrt_cpp_api)
# target_link_libraries(fine_grid_test PRIVATE CCCL::CCCL)
set_target_properties(fine_grid_test PROPERTIES CUDA_ARCHITECTURES "86")
