add_executable(kaizen_tests
    test_placeholder.cpp
    test_skeleton.cpp
    test_ik.cpp
    test_jacobian_ik.cpp
)
target_link_libraries(kaizen_tests PRIVATE
    kaizen_game
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(kaizen_tests)
