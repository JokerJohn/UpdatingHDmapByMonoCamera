################################################################################
# Part of CMake configuration for GEOS
#
# Copyright (C) 2018 Mateusz Loskot <mateusz@loskot.net>
#
# This is free software; you can redistribute and/or modify it under
# the terms of the GNU Lesser General Public Licence as published
# by the Free Software Foundation.
# See the COPYING file for more information.
################################################################################
cmake_minimum_required(VERSION 3.5)
project(geos_test)
find_package(GEOS 3.8 REQUIRED)
set(CMAKE_CXX_STANDARD 11)

add_executable(test_big_sweep_line_speed
    TestSweepLineSpeed.cpp
    GeometryTestFactory.cpp)
target_link_libraries(test_big_sweep_line_speed PRIVATE geos)
