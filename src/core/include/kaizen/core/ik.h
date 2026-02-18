#pragma once

#include "kaizen/core/math.h"
#include "kaizen/core/skeleton.h"
#include "kaizen/core/types.h"

#include <variant>
#include <vector>

namespace kaizen {

enum class JointLimitType : u8 { None, Hinge, Cone };

struct HingeLimit {
    Vec3 axis;       // hinge axis in parent-local space
    f32 min_angle;   // radians
    f32 max_angle;   // radians
};

struct ConeLimit {
    Vec3 axis;       // primary axis in parent-local space
    f32 half_angle;  // radians
};

using JointConstraint = std::variant<std::monostate, HingeLimit, ConeLimit>;

struct IKTarget {
    BoneId bone = INVALID_BONE;   // which bone this target controls
    Vec3 position{0.0f};
    f32 weight = 1.0f;            // 0-1, blend between rest and solved pose
    bool active = false;
};

struct IKSetup {
    std::vector<JointConstraint> constraints; // indexed by BoneId, size == bone_count

    static IKSetup build_humanoid();
};

struct IKSolverConfig {
    u32 max_iterations = 10;
    f32 tolerance = 0.001f;      // meters
    f32 root_mobility = 0.0f;    // 0 = pinned, 1 = fully mobile
};

void ik_solve(const Skeleton& skeleton, const IKSetup& setup,
              const std::vector<IKTarget>& targets, Pose& pose,
              const std::vector<Mat4>& world_transforms,
              const IKSolverConfig& config = {});

} // namespace kaizen
