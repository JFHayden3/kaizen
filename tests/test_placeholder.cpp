#include <catch2/catch_test_macros.hpp>
#include "kaizen/game/match_engine.h"

TEST_CASE("MatchEngine can be constructed and ticked", "[engine]") {
    kaizen::MatchEngine engine;
    engine.tick(1.0f / 60.0f);
    REQUIRE(true);
}
