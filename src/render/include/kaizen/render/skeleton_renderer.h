#pragma once

#include "kaizen/core/math.h"
#include "kaizen/core/skeleton.h"
#include "kaizen/render/debug_draw.h"

#include <vector>

namespace kaizen {

class SkeletonRenderer {
public:
    static void draw(const Skeleton& skeleton,
                     const std::vector<Mat4>& world_transforms,
                     DebugDraw& debug,
                     const Vec3& tint = Vec3{1.0f});

    static void draw_ground_grid(DebugDraw& debug,
                                 float size = 5.0f,
                                 float step = 0.5f);
};

} // namespace kaizen
