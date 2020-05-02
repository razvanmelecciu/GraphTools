cmake_minimum_required(VERSION 3.5)
project(main_test VERSION 1.0.0)

# Set variables
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(BOOST_VER 1.58.0)
set(SECTION_SEPARATOR "----------------------------------------------------------")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

# Add path to additional modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../modules")

# Output paths
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Options for definitions
add_definitions(-DBOOST_LOG_DYN_LINK)

# Find packages
find_package(Threads REQUIRED)
find_package(Boost ${BOOST_VER} COMPONENTS log)
find_package(TBB)
find_package(OpenMP)

# Debug messages
# ...

# Compile and assemble into a target
add_executable(main_test main_test.cpp tests.cpp)
target_link_libraries(main_test Threads::Threads)
if(Boost_FOUND)
    message(STATUS "Adding support for Boost libraries...")
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(main_test Boost::log)
endif()
if(TBB_FOUND)
    message(STATUS "Adding support for TBB libraries...")
    target_link_libraries(main_test tbb)
endif()
if(OpenMP_FOUND)
    message(STATUS "Adding support for OpenMP libraries...")
    target_link_libraries(main_test OpenMP::OpenMP_CXX)
endif()
message(STATUS "Adding support for Google Test/Mock libraries...")
target_link_libraries(main_test gtest gmock)
target_include_directories(main_test PUBLIC "${PROJECT_BINARY_DIR}")

# Log output paths
message(STATUS ${SECTION_SEPARATOR})
message(STATUS "Binaries Output path - Archive: " ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
message(STATUS "Binaries Output path - Library: " ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
message(STATUS "Binaries Output path - Runtime: " ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
message(STATUS ${SECTION_SEPARATOR})