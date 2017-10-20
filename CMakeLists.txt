project(PTAM-GS)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

set(CMAKE_CXX_FLAGS "-std=c++11")

# OpenCV
find_package( OpenCV 3.1 REQUIRED )
if(OpenCV_FOUND)
    include_directories( ${OpenCV_INCLUDE_DIRS} )
    link_libraries( ${OpenCV_LIBS} )
endif()

# Eigen
find_package(Eigen3 REQUIRED)
if(EIGEN3_FOUND)
    include_directories( ${EIGEN3_INCLUDE_DIR} )
endif()

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

add_executable(${PROJECT_NAME} ${SRC_LIST})

