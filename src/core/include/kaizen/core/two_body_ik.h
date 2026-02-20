#pragma once

#include "kaizen/core/ik.h"

#include <vector>

namespace kaizen {

struct ContactConstraint {
    BoneId bone_a = INVALID_BONE;  // bone on body A
    BoneId bone_b = INVALID_BONE;  // bone on body B
    f32 weight = 1.0f;
    bool active = true;
};

struct TwoBodyIKConfig {
    u32 max_iterations     = 10;
    f32 tolerance          = 0.001f;
    f32 damping            = 0.5f;
    f32 max_step           = 0.1f;
    bool root_translates_a = true;
    bool root_translates_b = true;
    f32 root_weight_a      = 0.3f;  // scales root translation effect (lower = prefer rotations)
    f32 root_weight_b      = 0.3f;
};

struct TwoBodyIKResult {
    u32 iterations = 0;
    f32 max_error  = 0.0f;
};

TwoBodyIKResult two_body_ik_solve(
    const Skeleton& skeleton,          // shared skeleton definition
    const IKSetup& setup_a,
    const IKSetup& setup_b,
    const std::vector<IKTarget>& targets_a,
    const std::vector<IKTarget>& targets_b,
    const std::vector<ContactConstraint>& contacts,
    Pose& pose_a, Pose& pose_b,
    const std::vector<Mat4>& world_a,
    const std::vector<Mat4>& world_b,
    const TwoBodyIKConfig& config = {});

} // namespace kaizen
