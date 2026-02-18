add_library(kaizen_render STATIC
    src/renderer.cpp
    src/debug_draw.cpp
    src/camera.cpp
    src/skeleton_renderer.cpp
)
target_include_directories(kaizen_render PUBLIC include)
target_link_libraries(kaizen_render PUBLIC kaizen_platform kaizen_core imgui)
