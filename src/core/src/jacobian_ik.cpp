#include "kaizen/core/jacobian_ik.h"
#include "ik_internal.h"
#include "dense_math.h"
#include "ik_shared.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace kaizen {

using detail::quat_rotate;
using detail::apply_constraint;

// --- Public API ---

u32 jacobian_ik_solve(const Skeleton& skeleton,
                      const IKSetup& setup,
                      const std::vector<IKTarget>& targets,
                      Pose& pose,
                      const std::vector<Mat4>& world_transforms,
                      const JacobianIKConfig& config) {

    // Filter active targets
    struct ActiveTarget {
        BoneId bone;
        Vec3 position;
        f32 weight;
    };
    std::vector<ActiveTarget> active;
    for (const auto& t : targets) {
        if (t.active && t.bone != INVALID_BONE && t.bone < skeleton.bone_count()) {
            active.push_back({t.bone, t.position, t.weight});
        }
    }
    if (active.empty()) return 0;

    u32 num_constraints = static_cast<u32>(active.size());
    u32 constraint_dims = num_constraints * 3; // 3 position components per target

    // Build DOF map
    DOFMap dof_map = build_dof_map(skeleton, targets, config.root_translates);
    if (dof_map.total_dofs == 0) return 0;

    // FK working buffers
    std::vector<Mat4> fk_world;
    std::vector<Quat> fk_rots;
    std::vector<Vec3> fk_pos;

    // Initial FK
    compute_fk(skeleton, pose, fk_world, fk_rots, fk_pos);

    // Iteration
    u32 iter = 0;
    for (; iter < config.max_iterations; ++iter) {
        // Compute residual (Δx = target - current)
        std::vector<f32> residual(constraint_dims);
        bool converged = true;
        for (u32 ci = 0; ci < num_constraints; ++ci) {
            Vec3 diff = active[ci].position - fk_pos[active[ci].bone];
            diff *= active[ci].weight;
            residual[ci * 3 + 0] = diff.x;
            residual[ci * 3 + 1] = diff.y;
            residual[ci * 3 + 2] = diff.z;

            f32 err = glm::length(active[ci].position - fk_pos[active[ci].bone]);
            if (err >= config.tolerance) converged = false;
        }
        if (converged) break;

        // Build Jacobian (constraint_dims x total_dofs)
        DenseMat J(constraint_dims, dof_map.total_dofs);

        for (u32 ci = 0; ci < num_constraints; ++ci) {
            BoneId target_bone = active[ci].bone;
            Vec3 target_world_pos = fk_pos[target_bone];

            // For each bone in the DOF map that is an ancestor of (or equal to) target_bone
            for (size_t bi = 0; bi < dof_map.bones.size(); ++bi) {
                BoneId dof_bone = dof_map.bones[bi];

                // Check if dof_bone is an ancestor of target_bone
                bool is_ancestor = false;
                BoneId cur = target_bone;
                while (cur != INVALID_BONE) {
                    if (cur == dof_bone) { is_ancestor = true; break; }
                    cur = skeleton.bones[cur].parent;
                }
                if (!is_ancestor) continue;

                Vec3 bone_world_pos = fk_pos[dof_bone];
                Vec3 r = target_world_pos - bone_world_pos;

                // Get parent world rotation for axis computation
                Quat parent_rot;
                BoneId parent = skeleton.bones[dof_bone].parent;
                if (parent == INVALID_BONE) {
                    parent_rot = Quat{1.0f, 0.0f, 0.0f, 0.0f};
                } else {
                    parent_rot = fk_rots[parent];
                }

                // 3 rotation DOFs: local X, Y, Z axes rotated to world frame
                u32 dof_start = dof_map.bone_dof_start[bi];
                Vec3 local_axes[3] = {
                    {1.0f, 0.0f, 0.0f},
                    {0.0f, 1.0f, 0.0f},
                    {0.0f, 0.0f, 1.0f}
                };

                for (u32 axis = 0; axis < 3; ++axis) {
                    Vec3 world_axis = quat_rotate(parent_rot, local_axes[axis]);
                    Vec3 contribution = glm::cross(world_axis, r);
                    contribution *= active[ci].weight;

                    J(ci * 3 + 0, dof_start + axis) = contribution.x;
                    J(ci * 3 + 1, dof_start + axis) = contribution.y;
                    J(ci * 3 + 2, dof_start + axis) = contribution.z;
                }
            }

            // Root translation DOFs: unit axes
            if (dof_map.has_root_translation) {
                // Check if root is ancestor of target
                bool root_is_ancestor = false;
                BoneId cur = target_bone;
                while (cur != INVALID_BONE) {
                    if (cur == dof_map.root_bone) { root_is_ancestor = true; break; }
                    cur = skeleton.bones[cur].parent;
                }
                if (root_is_ancestor) {
                    f32 w = active[ci].weight;
                    J(ci * 3 + 0, dof_map.root_trans_start + 0) = w;
                    J(ci * 3 + 1, dof_map.root_trans_start + 1) = w;
                    J(ci * 3 + 2, dof_map.root_trans_start + 2) = w;
                }
            }
        }

        // DLS solve: Δθ = J^T (JJ^T + λ²I)^{-1} Δx
        DenseMat Jt = mat_transpose(J);
        DenseMat JJt = mat_mul(J, Jt);
        mat_add_scaled_identity(JJt, config.damping * config.damping);

        std::vector<f32> y = solve_linear(JJt, residual);
        std::vector<f32> delta_theta = mat_vec_mul(Jt, y);

        // Apply rotation DOFs
        for (size_t bi = 0; bi < dof_map.bones.size(); ++bi) {
            BoneId bone = dof_map.bones[bi];
            u32 dof_start = dof_map.bone_dof_start[bi];

            Vec3 delta_rot(delta_theta[dof_start + 0],
                           delta_theta[dof_start + 1],
                           delta_theta[dof_start + 2]);

            // Clamp magnitude
            f32 mag = glm::length(delta_rot);
            if (mag > config.max_step) {
                delta_rot *= config.max_step / mag;
                mag = config.max_step;
            }

            if (mag > 1e-8f) {
                Vec3 axis = delta_rot / mag;
                Quat delta_q = glm::angleAxis(mag, axis);
                pose.transforms[bone].rotation = delta_q * pose.transforms[bone].rotation;
                pose.transforms[bone].rotation = glm::normalize(pose.transforms[bone].rotation);
            }

            // Enforce joint limits
            if (bone < static_cast<BoneId>(setup.constraints.size())) {
                pose.transforms[bone].rotation =
                    apply_constraint(setup.constraints[bone], pose.transforms[bone].rotation);
            }
        }

        // Apply root translation DOFs
        if (dof_map.has_root_translation) {
            Vec3 delta_trans(delta_theta[dof_map.root_trans_start + 0],
                             delta_theta[dof_map.root_trans_start + 1],
                             delta_theta[dof_map.root_trans_start + 2]);

            // Clamp translation step
            f32 trans_mag = glm::length(delta_trans);
            if (trans_mag > config.max_step) {
                delta_trans *= config.max_step / trans_mag;
            }

            pose.transforms[dof_map.root_bone].translation += delta_trans;
        }

        // Recompute FK for next iteration
        compute_fk(skeleton, pose, fk_world, fk_rots, fk_pos);
    }

    return iter;
}

} // namespace kaizen
