add_executable(kaizen_tests
    test_placeholder.cpp
)
target_link_libraries(kaizen_tests PRIVATE
    kaizen_game
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(kaizen_tests)
