#-------------------------------------------------------------------------------
# Project name: Configuration file for CMake
# File: CMakeLists.txt
# File encoding: UTF-8
# File created by: Radim Kriz (xkrizr03@stud.fit.vutbr.cz)
#
# Description: Cmake configuration file
#-------------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.6)

find_package(OpenCV REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pedantic -Werror -Wall")
endif ()

include_directories (${umf_cylinder_detector_SOURCE_DIR}/original_umf)

link_directories (${umf_cylinder_detector_BINARY_DIR}/original_umf)

set(SRCS
edges.cpp
edgels.cpp
fitting.cpp
ellipse_ransac.cpp
ransac.cpp
geometry_fundamentals.cpp
umf_wrapper.cpp
)

set(HDRS
edges.h
edgels.h
fitting.h
ellipse_ransac.h
ransac.h
geometry_fundamentals.h
umf_wrapper.h
)

# Main
add_executable(cylinder_detector main.cpp ${SRCS} ${HDRS})
target_link_libraries(cylinder_detector umfOriginal ${OpenCV_LIBS})

# Ellipse testing
#add_executable(ransac_test main_ransac_test.cpp ${SRCS} ${HDRS})
#taget_link_libraries(ransac_test ${OpenCV_LIBS})
