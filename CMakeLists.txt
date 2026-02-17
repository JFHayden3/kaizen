cmake_minimum_required(VERSION 3.24)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "")
project(kaizen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

option(KAIZEN_BUILD_SANDBOX "Build the ImGui sandbox" ON)
option(KAIZEN_BUILD_TESTS "Build tests" ON)
option(KAIZEN_BUILD_TOOLS "Build content pipeline tools" OFF)

include(cmake/Dependencies.cmake)

add_subdirectory(src/core)
add_subdirectory(src/game)
add_subdirectory(src/platform)
add_subdirectory(src/render)

add_subdirectory(apps/game)

if(KAIZEN_BUILD_SANDBOX)
    add_subdirectory(apps/sandbox)
endif()

if(KAIZEN_BUILD_TOOLS)
    add_subdirectory(apps/tools)
endif()

if(KAIZEN_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
