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
line_clustering.cpp
parabola_fitting.cpp
fitting_line.cpp
ellipse_clustering.cpp
)

set(HDRS
edges.h
edgels.h
fitting.h
ellipse_ransac.h
ransac.h
geometry_fundamentals.h
umf_wrapper.h
lines_clustering.h
standart_deviation.h
parabola_fitting.h
fitting_line.h
ellipse_clustering.h
)

# Ellipse version
add_executable(cylinder_detector main.cpp ${SRCS} ${HDRS})
target_link_libraries(cylinder_detector umfOriginal ${OpenCV_LIBS})

# Parabola testing
#add_executable(parabola_test parabola_fitting_test.cpp ${SRCS} ${HDRS})
#target_link_libraries(parabola_test umfOriginal ${OpenCV_LIBS})

# Parabola version
#add_executable(parabola_version main2.cpp ${SRCS} ${HDRS})
#target_link_libraries(parabola_version umfOriginal ${OpenCV_LIBS})

# Ellipse testing
#add_executable(ransac_test main_ransac_test.cpp ${SRCS} ${HDRS})
#target_link_libraries(ransac_test ${OpenCV_LIBS})
