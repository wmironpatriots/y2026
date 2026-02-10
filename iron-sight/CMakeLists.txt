cmake_minimum_required(VERSION 3.15)
project(IRONSIGHT CXX C)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # enable o3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # warn all

option(ENABLE_CLANG_TIDY "Enable clang-tidy for static analysis" ON)

find_package(wpilib 2026.2.1 REQUIRED)
find_package(OpenCV 4.13.0 REQUIRED)
find_package(apriltag 3.4.5 REQUIRED)
find_package(nlohmann_json 3.12.0 REQUIRED)

add_subdirectory(src)