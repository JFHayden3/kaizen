add_executable(kaizen_sandbox main.cpp)
target_link_libraries(kaizen_sandbox PRIVATE
    kaizen_game
    kaizen_platform
    kaizen_render
    imgui
)
