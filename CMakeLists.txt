cmake_minimum_required(VERSION 3.12)

# include the project setting file
include($ENV{DEVKIT_LOCATION}/cmake/pluginEntry.cmake)

# specify project name
set(PROJECT_NAME tvContact)

# set SOURCE_FILES
set(SOURCE_FILES
        Command.cpp
        )

# set linking libraries
set(LIBRARIES
        OpenMaya
        Foundation
        )

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/libraries)
set(EXECUTABLE_OUTPUT_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/libraries)

include_directories($ENV{DEVKIT_LOCATION}/include)

# Build plugin
add_library(tvContact SHARED plugin.cpp tvContact.cpp volumePreservation.cpp)
