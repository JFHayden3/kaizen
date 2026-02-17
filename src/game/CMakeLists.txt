add_library(kaizen_game STATIC
    src/match_engine.cpp
)
target_include_directories(kaizen_game PUBLIC include)
target_link_libraries(kaizen_game PUBLIC kaizen_core)
