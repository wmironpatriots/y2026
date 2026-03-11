cmake_minimum_required(VERSION 3.15)
project(IRONSIGHT CXX C)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3") # enable o3
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall") # warn all

option(ENABLE_CLANG_TIDY "Enable clang-tidy for static analysis" ON)

include(FetchContent)

FetchContent_Declare(
    Sleipnir
    GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
    GIT_TAG main
    EXCLUDE_FROM_ALL
    SYSTEM
)
FetchContent_MakeAvailable(Sleipnir)

# add_executable(lutGenerator LutGenerator.cc)
# target_link_libraries(lutGenerator PRIVATE Sleipnir::Sleipnir)
# set_target_properties(lutGenerator PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

add_executable(ex ex.cpp)
target_link_libraries(ex PRIVATE Sleipnir::Sleipnir)
set_target_properties(ex PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

