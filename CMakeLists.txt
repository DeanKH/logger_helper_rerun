cmake_minimum_required(VERSION 3.20)
project(logger_helper_rerun)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# find rerun_sdk, if not found download via FetchContent
find_package(rerun_sdk QUIET)
if(NOT rerun_sdk_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
    )
    FetchContent_MakeAvailable(rerun_sdk)
endif()

find_package(Eigen3 REQUIRED)


add_subdirectory(modules/logger_basic)
add_subdirectory(modules/logger_2d)
add_subdirectory(modules/logger_3d)

# if build examples, add subdirectory
option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()