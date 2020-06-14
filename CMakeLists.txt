cmake_minimum_required(VERSION 3.5)
project(main_app VERSION 1.0.0)
configure_file(config.h.in config.h)

# Consider Google Test/Mock to be globally installed
set(GOOGLE_TEST_GLOBAL_INSTALL TRUE)

add_subdirectory(test)
include_directories(src)

if (NOT GOOGLE_TEST_GLOBAL_INSTALL)
    message(STATUS "Searching for Google Test/Mock locally")
    add_subdirectory(lib/GoogleTest)
else()
    message(STATUS "Using Google Test/Mock globally. Make sure it's installed.")
    # apt install libgmock-dev
    # apt install libgtest-dev
endif()

# Set variables
#set(GCC_CUSTOM_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
#set(GCC_CUSTOM_LINK_FLAGS    "-lgcov")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(BOOST_VER 1.58.0)
set(SECTION_SEPARATOR "----------------------------------------------------------")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

# Add path to additional modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

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
message(STATUS ${SECTION_SEPARATOR})
message(STATUS "G++ Version: "${CMAKE_CXX_COMPILER})
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
message(STATUS "C++ Standard: " ${CMAKE_CXX_STANDARD})
if(Boost_FOUND)
    message(STATUS "Boost Version: " ${BOOST_VER})
endif()
if(TBB_FOUND)
    message(STATUS "TBB Version: " ${TBB_VERSION_MAJOR} ${TBB_VERSION.MINOR})
    message(STATUS "TBB Interface Version: " ${TBB_INTERFACE_VERSION})
endif()
if(OpenMP_FOUND)
    message(STATUS "OpenMP Spec Date: " ${OpenMP_CXX_SPEC_DATE})
    message(STATUS "OpenMP C++ Flags: " ${OpenMP_CXX_FLAGS})
endif()
message(STATUS ${SECTION_SEPARATOR})

# Append custom compile flags
if (GCC_CUSTOM_COMPILE_FLAGS)
    message(STATUS "Custom g++ compile Flags: " ${GCC_CUSTOM_COMPILE_FLAGS})
    set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_CUSTOM_COMPILE_FLAGS}")
endif()
if (GCC_CUSTOM_LINK_FLAGS)
    message(STATUS "Custom g++/ld link Flags: " ${GCC_CUSTOM_LINK_FLAGS})
    set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_CUSTOM_LINK_FLAGS}")
endif()

# Compile and assemble into a target
add_executable(main_app main_launch.cpp)
target_link_libraries(main_app Threads::Threads)
if(Boost_FOUND)
    message(STATUS "Adding support for Boost libraries...")
    include_directories(${Boost_INCLUDE_DIRS})
    target_link_libraries(main_app Boost::log)
endif()
if(TBB_FOUND)
    message(STATUS "Adding support for TBB libraries...")
    target_link_libraries(main_app tbb)
endif()
if(OpenMP_FOUND)
    message(STATUS "Adding support for OpenMP libraries...")
    target_link_libraries(main_app OpenMP::OpenMP_CXX)
endif()
target_include_directories(main_app PUBLIC "${PROJECT_BINARY_DIR}")

# Log output paths
message(STATUS ${SECTION_SEPARATOR})
message(STATUS "Binaries Output path - Archive: " ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY})
message(STATUS "Binaries Output path - Library: " ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
message(STATUS "Binaries Output path - Runtime: " ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
message(STATUS ${SECTION_SEPARATOR})