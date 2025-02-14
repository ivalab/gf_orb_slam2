cmake_minimum_required(VERSION 2.8)
project(gf_orb_slam2)

option(ENABLE_CLOSED_LOOP "Build Closed-Loop" ON)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    sensor_msgs
    cv_bridge
    image_transport
    tf
    tf2
    tf2_geometry_msgs
    tf2_ros
    geometry_msgs
    # for closed-loop evaluation of Good Graph only
    # trajectory_state_predictor
)

if(ENABLE_CLOSED_LOOP)
    add_definitions(-DENABLE_CLOSED_LOOP)
    find_package(catkin REQUIRED COMPONENTS
        rospy
        roscpp
        sensor_msgs
        cv_bridge
        image_transport
        tf
        tf2
        tf2_geometry_msgs
        tf2_ros
        geometry_msgs
        # for closed-loop evaluation of Good Graph only
        trajectory_state_predictor
    )
endif()

message("ROS version = " $ENV{ROS_DISTRO})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

IF(NOT DEFINED ORB_LIB_TYPE)
    SET (ORB_LIB_TYPE "SHARED") # "STATIC"
ENDIF()

IF(NOT DEFINED ENABLE_CUDA_IN_OPENCV)
    SET(ENABLE_CUDA_IN_OPENCV OFF)
ENDIF()
message("Enable cuda support: " ${ENABLE_CUDA_IN_OPENCV})


# Check architecture
IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    add_definitions("-DENABLE_SSE")
    MESSAGE("SSE flags: " ${SSE_FLAGS})
    # Optimized for Jetson TX2 only; refer to https://devtalk.nvidia.com/default/topic/1028179/jetson-tx2/gcc-options-for-tx2
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wl,--no-as-needed -pthread -fopenmp -Wall -O3 -ffast-math -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed -pthread -fopenmp -Wall -O3 -ffast-math -flto -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
ELSE()
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wl,--no-as-needed -pthread -fopenmp -Wall -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--no-as-needed -pthread -fopenmp -Wall -O3")
ENDIF()

# Add CUDA support
if(ENABLE_CUDA_IN_OPENCV)
    find_package(CUDA REQUIRED)
    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    SET(CUDA_HOST_COMPILER /usr/bin/g++)
    LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")
    #LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -Xcompiler -fpermissive")
endif()

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   if(ENABLE_CUDA_IN_OPENCV)
       set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
       set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -DUSE_NVTX")
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_NVTX")
   endif()
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   if(ENABLE_CUDA_IN_OPENCV)
       set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++0x")
       set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -DUSE_NVTX")
       set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_NVTX")
   endif()
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

IF(SLAM_P_P_SHORT_CXSPARSE)
        ADD_DEFINITIONS(-D__CXSPARSE_SHORT)
ENDIF(SLAM_P_P_SHORT_CXSPARSE)

IF(SLAM_P_P_SHORT_CHOLMOD)
        ADD_DEFINITIONS(-D__CHOLMOD_SHORT)
ENDIF(SLAM_P_P_SHORT_CHOLMOD)

IF(SLAM_P_P_FLAT_SYSTEM_ALIGNED_MEMORY)
        ADD_DEFINITIONS(-D__FLAT_SYSTEM_ALIGNED_MEMORY)
ENDIF(SLAM_P_P_FLAT_SYSTEM_ALIGNED_MEMORY)

# support for GPU BLAS
#IF(SLAM_P_P_GPU_BLAS)
#        ADD_DEFINITIONS(
#                -DGPU_BLAS
#                -D__CHANNEL_DESCRIPTOR_H__
#                -D__CUDA_RUNTIME_H__)
#ENDIF(SLAM_P_P_GPU_BLAS)

ADD_DEFINITIONS(-DCERES_NO_PROTOCOL_BUFFERS)

IF(SLAM_P_P_LINEAR_SOLVER_TYPE)
        ADD_DEFINITIONS("-D__LINEAR_SOLVER_OVERRIDE=${SLAM_P_P_LINEAR_SOLVER_TYPE}")
ENDIF(SLAM_P_P_LINEAR_SOLVER_TYPE)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# check environment first 
if (NOT DEFINED ENV{GF_ORB_SLAM2_ROOT})
    message(FATAL_ERROR "env variable: GF_ORB_SLAM2_ROOT is NOT defined, please source export_env_variables.bash")
endif()

# customize opencv dir
find_package(OpenCV 4 REQUIRED)
MESSAGE("OpenCV include dir: " ${OpenCV_INCLUDE_DIRS})
MESSAGE("OpenCV libs: " ${OpenCV_LIBS})

# customize eigen dir
set( EIGEN3_INCLUDE_DIR $ENV{GF_ORB_SLAM2_ROOT}/eigen33/include/eigen3/ )
MESSAGE("EIGEN include dir: " ${EIGEN3_INCLUDE_DIR})

# customize pangolin dir
set(Pangolin_INCLUDE_DIRS $ENV{GF_ORB_SLAM2_ROOT}/Pangolin/include/)
set(Pangolin_LIBRARIES    $ENV{GF_ORB_SLAM2_ROOT}/Pangolin/lib/libpangolin.so)
MESSAGE("Pangolin include dir: " ${Pangolin_INCLUDE_DIRS})

# customize openblas dir
set(OpenBLAS_LIBRARIES $ENV{GF_ORB_SLAM2_ROOT}/OpenBLAS/lib/libopenblas.so)
MESSAGE("OpenBLAS libs:" ${OpenBLAS_LIBRARIES})

# customize armadillo dir
set(ARMADILLO_INCLUDE_DIR $ENV{GF_ORB_SLAM2_ROOT}/armadillo/include)
set(ARMADILLO_LIBRARIES   $ENV{GF_ORB_SLAM2_ROOT}/armadillo/lib/libarmadillo.so)
MESSAGE("ARMADILLO libs: " ${ARMADILLO_LIBRARIES})

# customize gtest dir
set(GTest_INCLUDE_DIR $ENV{GF_ORB_SLAM2_ROOT}/gtest/include)
set(GTest_LIBRARIES $ENV{GF_ORB_SLAM2_ROOT}/gtest/lib/libgtest.a)
MESSAGE("gtest libs: " ${GTest_LIBRARIES})

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    MESSAGE("ccache:" ${CCACHE_FOUND})
else (CCACHE_FOUND)
    MESSAGE("No ccache detected")
endif(CCACHE_FOUND)

# find GFlags
set(GFLAGS_INCLUDE_DIR $ENV{GF_ORB_SLAM2_ROOT}/gflags/include)
set(GFLAGS_LIBRARIES $ENV{GF_ORB_SLAM2_ROOT}/gflags/lib/libgflags.a)
# find_package(GFlags REQUIRED)
MESSAGE("gflags include dirs: " ${GFLAGS_INCLUDE_DIR})

# find boost
find_package(Boost REQUIRED COMPONENTS thread system filesystem)
MESSAGE("Boost include dir: " ${Boost_INCLUDE_DIRS})
MESSAGE("Boost lib: " ${Boost_LIBRARIES})

# include directories
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/include
    ${PROJECT_SOURCE_DIR}/Thirdparty/sse2neon/
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${GTest_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if(ENABLE_CUDA_IN_OPENCV)
    cuda_include_directories(
      ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc
    )
    cuda_add_library(${PROJECT_NAME} ${ORB_LIB_TYPE}
        include/Util_cuda.hpp
        include/Util.hpp
        src/System.cc
        src/ChArUco.cc
        src/Observability.cc
        src/Hashing.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/MapPublisher.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        src/cuda/Allocator_gpu.cu
        src/cuda/Fast_gpu.cu
        src/cuda/Orb_gpu.cu
        src/cuda/Cuda.cu
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/src/good_graph_testbed/BAOptimizer.cpp
    )
    IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
        cuda_add_library(${PROJECT_NAME} ${ORB_LIB_TYPE} src/FAST_NEON.cc)
    ENDIF()
else()
    set(SRCS
        include/Util.hpp
        src/System.cc
        src/ChArUco.cc
        src/Hashing.cc
        src/Observability.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Map.cc
        src/MapDrawer.cc
        src/MapPublisher.cc
        src/Optimizer.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/src/good_graph_testbed/BAOptimizer.cpp
    )
    IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
        set(SRCS ${SRCS} src/FAST_NEON.cc)
    ENDIF()
    add_library(${PROJECT_NAME} ${ORB_LIB_TYPE} ${SRCS})
endif()

# SLAM ++ header files
FILE(GLOB_RECURSE SLAM_EXAMPLE_HEADERS
    ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/include/good_graph_testbed/*.h
    ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/include/slam/*.h)

if (${ORB_LIB_TYPE} STREQUAL "STATIC")
    set(LIBS
        ${GFLAGS_LIBRARIES}
        ${OpenCV_LIBS}
        # ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${OpenBLAS_LIBRARIES}
        ${ARMADILLO_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.a
        #libtrajectory_state_predictor.a
    )
    SET(SLAM_PLUS_PLUS_LIBRARIES
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libeigen.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcxsparse.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcsparse.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcholmod.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcamd_dlong.a
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libslampp_base.a
    )
else()
    set(LIBS
        GL
        glut
        GLU
        GLEW
        ${GFLAGS_LIBRARIES}
        ${OpenCV_LIBS}
        # ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${OpenBLAS_LIBRARIES}
        ${ARMADILLO_LIBRARIES}
        ${Boost_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        #trajectory_state_predictor
    )
    SET(SLAM_PLUS_PLUS_LIBRARIES
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libeigen.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcxsparse.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcsparse.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcholmod.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libcamd_dlong.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/SLAM++/lib/libslampp_base.so
    )
endif()

if(ENABLE_CUDA_IN_OPENCV)
    target_link_libraries(${PROJECT_NAME}
        ${LIBS}
        ${catkin_LIBRARIES}
        nvToolsExt
        ${SLAM_PLUS_PLUS_LIBRARIES})
else()
    target_link_libraries(${PROJECT_NAME}
        GL
        glut
        GLU
        ${LIBS}
        ${catkin_LIBRARIES}
        ${SLAM_PLUS_PLUS_LIBRARIES})
endif()
target_compile_options(${PROJECT_NAME} PRIVATE "-w")


# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/tools)
add_executable(bin_vocabulary tools/bin_vocabulary.cc)
target_link_libraries(bin_vocabulary ${PROJECT_NAME})


# Build ROS Nodes

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/ROS/GF_ORB_SLAM2)

# Node for monocular camera
add_executable(Mono Examples/ROS/GF_ORB_SLAM2/src/ros_mono.cc)

target_link_libraries(Mono
${LIBS}
${catkin_LIBRARIES}
${PROJECT_NAME}
)

# Node for monocular camera (Augmented Reality Demo)
#add_executable(MonoAR
#src/AR/ros_mono_ar.cc
#src/AR/ViewerAR.h
#src/AR/ViewerAR.cc
#)

#target_link_libraries(MonoAR
#${LIBS}
#)

# Node for stereo camera
add_executable(Stereo Examples/ROS/GF_ORB_SLAM2/src/ros_stereo.cc)
    # src/path_smoothing_ros/cubic_spline_interpolator.cpp)

target_link_libraries(Stereo
${LIBS}
${catkin_LIBRARIES}
${PROJECT_NAME}
)

# Node for RGB-D camera
add_executable(RGBD Examples/ROS/GF_ORB_SLAM2/src/ros_rgbd.cc)

# string(REPLACE "/opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1;" "" catkin_LIBRARIES "${catkin_LIBRARIES}")
MESSAGE("catkin_LIBRARIES: " ${catkin_LIBRARIES})

target_link_libraries(RGBD
${LIBS}
${catkin_LIBRARIES}
${PROJECT_NAME}
)

# build non-ROS node
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)
add_executable(stereo_euroc Examples/Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc
    ${PROJECT_NAME}
)

# Build test cases

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

add_executable(testObs
    ${SRCS}
    ./test/test_Kine_1.cpp
    ./test/test_Kine_2.cpp
    ./test/test_Jacobian.cpp
#    ./test/test_Greedy.cpp
#    ./test/test_GoodMap.cpp
    ./test/test_Stereo.cpp
#    ./test/test_MapIO.cpp
    ./test/main.cpp
    )
target_link_libraries(testObs
    GL
    glut
    GLU
    ${PROJECT_NAME}
    ${GTest_LIBRARIES}
    ${SLAM_PLUS_PLUS_LIBRARIES}
    ${OpenBLAS_LIBRARIES}
    ${ARMADILLO_LIBRARIES}
    ${GTest_LIBRARIES}
)

# Build exec for Jetson TX2
#unset(CMAKE_RUNTIME_OUTPUT_DIRECTORY)
#add_subdirectory(gpu)