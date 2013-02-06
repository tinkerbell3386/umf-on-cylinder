#-------------------------------------------------------------------------------
# Project name: Configuration file for CMake
# File: CMakeLists.txt
# File encoding: UTF-8
# File created by: Radim Kriz (xkrizr03@stud.fit.vutbr.cz)
#
# Description: Cmake configuration file
#-------------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.6)

project(umf_cylinder_detector)

add_subdirectory (original_umf)
add_subdirectory (cylinder_detector)
