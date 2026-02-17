add_library(kaizen_core STATIC
    src/core.cpp
)
target_include_directories(kaizen_core PUBLIC include)
target_link_libraries(kaizen_core PUBLIC yaml-cpp::yaml-cpp)
