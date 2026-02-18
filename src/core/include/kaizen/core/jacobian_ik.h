#pragma once

#include "kaizen/core/ik.h"

namespace kaizen {

struct JacobianIKConfig {
    u32 max_iterations   = 20;
    f32 tolerance        = 0.001f;   // meters, early-out per target
    f32 damping          = 0.5f;     // lambda for DLS regularization
    f32 max_step         = 0.1f;     // clamp per-iteration angular change (radians)
    bool root_translates = true;     // root bone gets 3 translation DOFs
};

// Jacobian-based IK solver operating in joint-angle space.
// Returns number of iterations performed.
// Modifies pose in-place — caller maintains persistence across frames.
u32 jacobian_ik_solve(const Skeleton& skeleton,
                      const IKSetup& setup,
                      const std::vector<IKTarget>& targets,
                      Pose& pose,
                      const std::vector<Mat4>& world_transforms,
                      const JacobianIKConfig& config = {});

} // namespace kaizen
