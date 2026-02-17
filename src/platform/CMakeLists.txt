add_library(kaizen_platform STATIC
    src/window.cpp
)
target_include_directories(kaizen_platform PUBLIC include)
target_link_libraries(kaizen_platform PUBLIC SDL2::SDL2-static)

find_package(OpenGL REQUIRED)
target_link_libraries(kaizen_platform PUBLIC OpenGL::GL)
