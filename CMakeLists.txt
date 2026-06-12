cmake_minimum_required(VERSION 3.5)
project(ORB_SLAM3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O0 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O0 -g -std=c++14")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -std=c++14")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin)
find_package(realsense2)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
#${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/ForcedPoseInit.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/FeatureExtractor.cc
src/AkazeExtractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h)

add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
#${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
-lboost_serialization
-lcrypto
)


# Build examples

#Monocular examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/localization_service)

add_executable(localization_service_host
        localization_service/src/localization_service_host.cc
        localization_service/src/slam_state.cc
        localization_service/src/ingest_queue.cc
        localization_service/src/espnode_source.cc
        localization_service/src/calibration_manager.cc
        localization_service/src/preprocessor.cc
        localization_service/src/web_server.cc)
target_include_directories(localization_service_host PRIVATE
        localization_service/include)
target_link_libraries(localization_service_host ${PROJECT_NAME} -lGL)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

add_executable(inspect_osa
        Examples/Monocular/inspect_osa.cc)
target_link_libraries(inspect_osa ${PROJECT_NAME} -lGL -lboost_serialization)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/localization_service/tools)

add_executable(osa_convert
        localization_service/tools/osa_convert.cc)
target_link_libraries(osa_convert ${PROJECT_NAME} -lGL -lboost_serialization)


# Install targets for AppImage packaging
# Set CMAKE_INSTALL_PREFIX to the AppDir path when building for AppImage:
#   cmake --install <build_dir> --prefix /path/to/AppDir
# Use FILES/PROGRAMS (not TARGETS) to skip CMake's RPATH rewriting, which
# fails on this toolchain due to a trailing-colon bug in cmake_install.cmake.
install(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/libORB_SLAM3.so
        DESTINATION usr/lib)

install(FILES
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        DESTINATION usr/lib)

install(PROGRAMS
        ${PROJECT_SOURCE_DIR}/localization_service/localization_service_host
        DESTINATION .)

install(DIRECTORY localization_service/html/
        DESTINATION html)

# AppRun resolves APPDIR_LIBC_LINKER_PATH relative to the AppDir root, so
# lib64/ld-linux-x86-64.so.2 must exist there.  appimage-builder moves the
# libc6 package files into runtime/compat/ and leaves AppDir/lib64/ empty,
# so we install the system linker here explicitly.
find_file(LD_LINUX_SO ld-linux-x86-64.so.2
    PATHS /lib/x86_64-linux-gnu /lib64
    NO_DEFAULT_PATH)
if(LD_LINUX_SO)
    install(PROGRAMS ${LD_LINUX_SO} DESTINATION lib64)
endif()
