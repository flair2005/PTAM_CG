project(PTAM-GS)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -std=c++11 -O3 -Wall -g")

# OpenCV
find_package( OpenCV REQUIRED COMPONENTS opencv_core opencv_imgcodecs opencv_imgproc opencv_features2d opencv_calib3d)
if(OpenCV_FOUND)
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    link_libraries( ${OpenCV_LIBS} )
endif()

## Eigen
#find_package(Eigen3 REQUIRED)
#if(EIGEN3_FOUND)
#    include_directories( ${EIGEN3_INCLUDE_DIR} )
#endif()

# Sophus
find_package( Sophus REQUIRED )
if(Sophus_FOUND)
    include_directories( ${Sophus_INCLUDE_DIRS} )
    link_libraries( ${Sophus_LIBRARIES} )
endif()

# OpenGL
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    link_libraries( ${OPENGL_LIBRARY} )
endif()

# GLEW
find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    link_libraries(${GLEW_LIBRARIES})
endif()

# Pangolin
find_package( Pangolin )
if(Pangolin_FOUND)
    include_directories( ${Pangolin_INCLUDE_DIRS} )
    link_libraries( ${Pangolin_LIBRARIES} )
endif()

## JsonCpp
link_libraries(jsoncpp)

################ PTAM-GS ################
include_directories(${PROJECT_SOURCE_DIR}/include)
set( SRC_LIST_PTAM
    ${PROJECT_SOURCE_DIR}/src/System.cpp
    ${PROJECT_SOURCE_DIR}/src/VideoSource.cpp
    ${PROJECT_SOURCE_DIR}/src/GLWindowPangolin.cpp
    ${PROJECT_SOURCE_DIR}/src/KeyFrame.cpp
    ${PROJECT_SOURCE_DIR}/src/Tracker.cpp
    ${PROJECT_SOURCE_DIR}/src/MapMaker.cpp
    ${PROJECT_SOURCE_DIR}/src/ImgProc.cpp
    ${PROJECT_SOURCE_DIR}/src/MiniPatch.cpp
    ${PROJECT_SOURCE_DIR}/src/ATANCamera.cpp
    ${PROJECT_SOURCE_DIR}/src/Feature2dDetector.cpp
    ${PROJECT_SOURCE_DIR}/src/Homography.cpp )
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/main.cpp ${SRC_LIST_PTAM})

################ gtest ################
enable_testing()
find_package(GTest REQUIRED)
if(GTest_FOUND)
    include_directories(${GTEST_INCLUDE_DIRS})
endif()
ADD_SUBDIRECTORY(UnitTest)
ADD_TEST(NAME ./UnitTest/UnitTest.run COMMAND ./UnitTest/UnitTest.run)
