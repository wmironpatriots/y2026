cmake_minimum_required(VERSION 3.15)

include_directories(${CMAKE_SOURCE_DIR})

if(ENABLE_CLANG_TIDY)
    set(CMAKE_CXX_CLANG_TIDY
        clang-tidy;
        -checks=modernize-*,performance-*;
        -warnings-as-errors=*;
    )
endif()

add_subdirectory(test)
add_subdirectory(utils)
add_subdirectory(camera)
add_subdirectory(localization)

add_executable(main main.cc)
target_link_directories(main PRIVATE ${OpenCV_LIBS} apriltag ntcore nlohmann_json::nlohmann_json)

set(CMAKE_CXX_CLANG_TIDY "")
