#include "kaizen/core/two_body_ik.h"
#include "ik_internal.h"
#include "dense_math.h"
#include "ik_shared.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace kaizen {

using detail::quat_rotate;
using detail::apply_constraint;

// Check if ancestor_bone is an ancestor of (or equal to) bone in skeleton
static bool is_ancestor_of(const Skeleton& skeleton, BoneId ancestor_bone, BoneId bone) {
    BoneId cur = bone;
    while (cur != INVALID_BONE) {
        if (cur == ancestor_bone) return true;
        cur = skeleton.bones[cur].parent;
    }
    return false;
}

static constexpr f32 SINGULARITY_THRESHOLD = 0.02f; // column norm below this triggers nudge
static constexpr f32 NUDGE_ANGLE = 0.05f;           // radians of perturbation

// Nudge joints near kinematic singularity (lever arm collinear with all rotation axes).
// Applies a small deterministic rotation using a fixed off-axis direction per bone.
static void nudge_singular_joints(const Skeleton& skeleton,
                                  const DOFMap& map,
                                  const std::vector<Quat>& fk_rots,
                                  const std::vector<Vec3>& fk_pos,
                                  const std::vector<BoneId>& target_bones,
                                  Pose& pose) {
    for (size_t bi = 0; bi < map.bones.size(); ++bi) {
        BoneId dof_bone = map.bones[bi];

        // Check if this bone is an ancestor of any target
        bool relevant = false;
        for (BoneId tb : target_bones) {
            BoneId cur = tb;
            while (cur != INVALID_BONE) {
                if (cur == dof_bone) { relevant = true; break; }
                cur = skeleton.bones[cur].parent;
            }
            if (relevant) break;
        }
        if (!relevant) continue;

        // For each target this bone affects, check lever arm magnitude
        for (BoneId tb : target_bones) {
            if (!is_ancestor_of(skeleton, dof_bone, tb)) continue;

            Vec3 r = fk_pos[tb] - fk_pos[dof_bone];
            f32 r_len = glm::length(r);
            if (r_len < 0.01f) continue; // too close, skip

            // Get parent rotation for world-space axes
            Quat parent_rot;
            BoneId parent = skeleton.bones[dof_bone].parent;
            parent_rot = (parent == INVALID_BONE) ? Quat{1.0f, 0.0f, 0.0f, 0.0f} : fk_rots[parent];

            // Check if all 3 rotation axes produce near-zero cross products with r
            Vec3 local_axes[3] = {{1,0,0}, {0,1,0}, {0,0,1}};
            f32 max_col_mag = 0.0f;
            for (int a = 0; a < 3; ++a) {
                Vec3 world_axis = quat_rotate(parent_rot, local_axes[a]);
                f32 col_mag = glm::length(glm::cross(world_axis, r));
                if (col_mag > max_col_mag) max_col_mag = col_mag;
            }

            if (max_col_mag < SINGULARITY_THRESHOLD * r_len) {
                // Near singularity: nudge this joint off-axis
                // Use a deterministic perpendicular direction
                Vec3 r_norm = r / r_len;
                Vec3 perp = glm::cross(r_norm, Vec3{0.0f, 1.0f, 0.0f});
                if (glm::length(perp) < 0.1f)
                    perp = glm::cross(r_norm, Vec3{1.0f, 0.0f, 0.0f});
                perp = glm::normalize(perp);

                Quat nudge = glm::angleAxis(NUDGE_ANGLE, perp);
                pose.transforms[dof_bone].rotation =
                    nudge * pose.transforms[dof_bone].rotation;
                pose.transforms[dof_bone].rotation =
                    glm::normalize(pose.transforms[dof_bone].rotation);
                break; // only nudge once per bone
            }
        }
    }
}

// Fill Jacobian rotation columns for a single body's contribution to a constraint row.
// sign: +1 or -1 depending on whether moving this body increases or decreases the error.
static void fill_rotation_columns(const Skeleton& skeleton,
                                  const DOFMap& map,
                                  const std::vector<Quat>& fk_rots,
                                  const std::vector<Vec3>& fk_pos,
                                  BoneId target_bone,
                                  Vec3 target_world_pos,
                                  u32 dof_offset,    // column offset in unified Jacobian
                                  u32 row_start,     // first row of this constraint (3 rows)
                                  f32 weight,
                                  f32 sign,
                                  DenseMat& J) {
    for (size_t bi = 0; bi < map.bones.size(); ++bi) {
        BoneId dof_bone = map.bones[bi];
        if (!is_ancestor_of(skeleton, dof_bone, target_bone)) continue;

        Vec3 bone_world_pos = fk_pos[dof_bone];
        Vec3 r = target_world_pos - bone_world_pos;

        Quat parent_rot;
        BoneId parent = skeleton.bones[dof_bone].parent;
        if (parent == INVALID_BONE) {
            parent_rot = Quat{1.0f, 0.0f, 0.0f, 0.0f};
        } else {
            parent_rot = fk_rots[parent];
        }

        u32 col_start = dof_offset + map.bone_dof_start[bi];
        Vec3 local_axes[3] = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f}
        };

        for (u32 axis = 0; axis < 3; ++axis) {
            Vec3 world_axis = quat_rotate(parent_rot, local_axes[axis]);
            Vec3 contribution = sign * weight * glm::cross(world_axis, r);
            J(row_start + 0, col_start + axis) = contribution.x;
            J(row_start + 1, col_start + axis) = contribution.y;
            J(row_start + 2, col_start + axis) = contribution.z;
        }
    }
}

// Apply delta_theta rotation and translation DOFs to a single body's pose.
// root_weight must match the Jacobian column scaling so the applied translation
// is consistent with what the solver predicted.
static void apply_deltas(const Skeleton& skeleton,
                         const IKSetup& setup,
                         const DOFMap& map,
                         const std::vector<f32>& delta_theta,
                         u32 dof_offset,
                         f32 max_step,
                         f32 root_weight,
                         Pose& pose) {
    // Apply rotation DOFs
    for (size_t bi = 0; bi < map.bones.size(); ++bi) {
        BoneId bone = map.bones[bi];
        u32 dof_start = dof_offset + map.bone_dof_start[bi];

        Vec3 delta_rot(delta_theta[dof_start + 0],
                       delta_theta[dof_start + 1],
                       delta_theta[dof_start + 2]);

        f32 mag = glm::length(delta_rot);
        if (mag > max_step) {
            delta_rot *= max_step / mag;
            mag = max_step;
        }

        if (mag > 1e-8f) {
            Vec3 axis = delta_rot / mag;
            Quat delta_q = glm::angleAxis(mag, axis);
            pose.transforms[bone].rotation = delta_q * pose.transforms[bone].rotation;
            pose.transforms[bone].rotation = glm::normalize(pose.transforms[bone].rotation);
        }

        if (bone < static_cast<BoneId>(setup.constraints.size())) {
            pose.transforms[bone].rotation =
                apply_constraint(setup.constraints[bone], pose.transforms[bone].rotation);
        }
    }

    // Apply root translation DOFs (scaled by root_weight to match Jacobian columns)
    if (map.has_root_translation) {
        u32 rt_start = dof_offset + map.root_trans_start;
        Vec3 delta_trans(delta_theta[rt_start + 0],
                         delta_theta[rt_start + 1],
                         delta_theta[rt_start + 2]);
        delta_trans *= root_weight;

        f32 trans_mag = glm::length(delta_trans);
        if (trans_mag > max_step) {
            delta_trans *= max_step / trans_mag;
        }

        pose.transforms[map.root_bone].translation += delta_trans;
    }
}

TwoBodyIKResult two_body_ik_solve(
    const Skeleton& skeleton,
    const IKSetup& setup_a,
    const IKSetup& setup_b,
    const std::vector<IKTarget>& targets_a,
    const std::vector<IKTarget>& targets_b,
    const std::vector<ContactConstraint>& contacts,
    Pose& pose_a, Pose& pose_b,
    const std::vector<Mat4>& /*world_a*/,
    const std::vector<Mat4>& /*world_b*/,
    const TwoBodyIKConfig& config) {

    TwoBodyIKResult result;

    // Collect active world targets per body
    struct ActiveTarget {
        BoneId bone;
        Vec3 position;
        f32 weight;
    };
    std::vector<ActiveTarget> active_a, active_b;
    for (const auto& t : targets_a) {
        if (t.active && t.bone != INVALID_BONE && t.bone < skeleton.bone_count())
            active_a.push_back({t.bone, t.position, t.weight});
    }
    for (const auto& t : targets_b) {
        if (t.active && t.bone != INVALID_BONE && t.bone < skeleton.bone_count())
            active_b.push_back({t.bone, t.position, t.weight});
    }

    // Collect active contacts
    struct ActiveContact {
        BoneId bone_a, bone_b;
        f32 weight;
    };
    std::vector<ActiveContact> active_contacts;
    for (const auto& c : contacts) {
        if (c.active && c.bone_a != INVALID_BONE && c.bone_b != INVALID_BONE &&
            c.bone_a < skeleton.bone_count() && c.bone_b < skeleton.bone_count())
            active_contacts.push_back({c.bone_a, c.bone_b, c.weight});
    }

    if (active_a.empty() && active_b.empty() && active_contacts.empty()) return result;

    // Build DOF maps per body.
    // Each body's DOF map needs targets from both world targets AND contact bones.
    // We create synthetic IKTarget entries from contact bones.
    auto make_seed_targets = [](const std::vector<ActiveTarget>& world_targets,
                                const std::vector<ActiveContact>& contacts,
                                bool is_body_a) {
        std::vector<IKTarget> seeds;
        for (const auto& t : world_targets) {
            seeds.push_back({.bone = t.bone, .active = true});
        }
        for (const auto& c : contacts) {
            seeds.push_back({.bone = is_body_a ? c.bone_a : c.bone_b, .active = true});
        }
        return seeds;
    };

    auto seeds_a = make_seed_targets(active_a, active_contacts, true);
    auto seeds_b = make_seed_targets(active_b, active_contacts, false);

    DOFMap map_a = build_dof_map(skeleton, seeds_a, config.root_translates_a);
    DOFMap map_b = build_dof_map(skeleton, seeds_b, config.root_translates_b);

    if (map_a.total_dofs == 0 && map_b.total_dofs == 0) return result;

    u32 offset_b = map_a.total_dofs;
    u32 total_dofs = map_a.total_dofs + map_b.total_dofs;

    u32 num_a = static_cast<u32>(active_a.size());
    u32 num_b = static_cast<u32>(active_b.size());
    u32 num_contacts = static_cast<u32>(active_contacts.size());
    u32 constraint_dims = (num_a + num_b + num_contacts) * 3;

    if (constraint_dims == 0) return result;

    // Collect target bones per body for singularity detection
    std::vector<BoneId> target_bones_a, target_bones_b;
    for (const auto& t : active_a) target_bones_a.push_back(t.bone);
    for (const auto& c : active_contacts) target_bones_a.push_back(c.bone_a);
    for (const auto& t : active_b) target_bones_b.push_back(t.bone);
    for (const auto& c : active_contacts) target_bones_b.push_back(c.bone_b);

    // FK working buffers
    std::vector<Mat4> fk_world_a, fk_world_b;
    std::vector<Quat> fk_rots_a, fk_rots_b;
    std::vector<Vec3> fk_pos_a, fk_pos_b;

    compute_fk(skeleton, pose_a, fk_world_a, fk_rots_a, fk_pos_a);
    compute_fk(skeleton, pose_b, fk_world_b, fk_rots_b, fk_pos_b);

    // Nudge joints near kinematic singularity before first iteration
    nudge_singular_joints(skeleton, map_a, fk_rots_a, fk_pos_a, target_bones_a, pose_a);
    nudge_singular_joints(skeleton, map_b, fk_rots_b, fk_pos_b, target_bones_b, pose_b);
    compute_fk(skeleton, pose_a, fk_world_a, fk_rots_a, fk_pos_a);
    compute_fk(skeleton, pose_b, fk_world_b, fk_rots_b, fk_pos_b);

    // Iteration
    for (; result.iterations < config.max_iterations; ++result.iterations) {
        // Build residual
        std::vector<f32> residual(constraint_dims);
        result.max_error = 0.0f;
        u32 row = 0;

        // Region 1: Body A world targets
        for (u32 ci = 0; ci < num_a; ++ci) {
            Vec3 diff = active_a[ci].position - fk_pos_a[active_a[ci].bone];
            f32 err = glm::length(diff);
            if (err > result.max_error) result.max_error = err;
            diff *= active_a[ci].weight;
            residual[row + 0] = diff.x;
            residual[row + 1] = diff.y;
            residual[row + 2] = diff.z;
            row += 3;
        }

        // Region 2: Body B world targets
        for (u32 ci = 0; ci < num_b; ++ci) {
            Vec3 diff = active_b[ci].position - fk_pos_b[active_b[ci].bone];
            f32 err = glm::length(diff);
            if (err > result.max_error) result.max_error = err;
            diff *= active_b[ci].weight;
            residual[row + 0] = diff.x;
            residual[row + 1] = diff.y;
            residual[row + 2] = diff.z;
            row += 3;
        }

        // Region 3: Contact constraints
        // Gap = pos(B.bone_b) - pos(A.bone_a), target = 0
        // Residual = target - gap = -(pos_b - pos_a) = pos_a - pos_b
        for (u32 ci = 0; ci < num_contacts; ++ci) {
            Vec3 pos_a_bone = fk_pos_a[active_contacts[ci].bone_a];
            Vec3 pos_b_bone = fk_pos_b[active_contacts[ci].bone_b];
            Vec3 gap = pos_b_bone - pos_a_bone;
            f32 err = glm::length(gap);
            if (err > result.max_error) result.max_error = err;
            Vec3 r = -gap * active_contacts[ci].weight;
            residual[row + 0] = r.x;
            residual[row + 1] = r.y;
            residual[row + 2] = r.z;
            row += 3;
        }

        if (result.max_error < config.tolerance) break;

        // Build Jacobian (constraint_dims x total_dofs)
        DenseMat J(constraint_dims, total_dofs);
        row = 0;

        // Region 1: Body A world targets — columns in [0, map_a.total_dofs)
        for (u32 ci = 0; ci < num_a; ++ci) {
            BoneId target_bone = active_a[ci].bone;
            Vec3 target_pos = fk_pos_a[target_bone];
            f32 w = active_a[ci].weight;

            fill_rotation_columns(skeleton, map_a, fk_rots_a, fk_pos_a,
                                  target_bone, target_pos, 0, row, w, 1.0f, J);

            // Root translation A
            if (map_a.has_root_translation && is_ancestor_of(skeleton, map_a.root_bone, target_bone)) {
                u32 rt = map_a.root_trans_start;
                f32 rw = w * config.root_weight_a;
                J(row + 0, rt + 0) = rw;
                J(row + 1, rt + 1) = rw;
                J(row + 2, rt + 2) = rw;
            }
            row += 3;
        }

        // Region 2: Body B world targets — columns in [offset_b, offset_b + map_b.total_dofs)
        for (u32 ci = 0; ci < num_b; ++ci) {
            BoneId target_bone = active_b[ci].bone;
            Vec3 target_pos = fk_pos_b[target_bone];
            f32 w = active_b[ci].weight;

            fill_rotation_columns(skeleton, map_b, fk_rots_b, fk_pos_b,
                                  target_bone, target_pos, offset_b, row, w, 1.0f, J);

            // Root translation B
            if (map_b.has_root_translation && is_ancestor_of(skeleton, map_b.root_bone, target_bone)) {
                u32 rt = offset_b + map_b.root_trans_start;
                f32 rw = w * config.root_weight_b;
                J(row + 0, rt + 0) = rw;
                J(row + 1, rt + 1) = rw;
                J(row + 2, rt + 2) = rw;
            }
            row += 3;
        }

        // Region 3: Contact constraints
        // Error = pos(B.bone_b) - pos(A.bone_a)
        // Body A ancestors of bone_a get NEGATIVE sign (moving A toward B reduces error)
        // Body B ancestors of bone_b get POSITIVE sign
        for (u32 ci = 0; ci < num_contacts; ++ci) {
            BoneId bone_a = active_contacts[ci].bone_a;
            BoneId bone_b = active_contacts[ci].bone_b;
            f32 w = active_contacts[ci].weight;

            Vec3 pos_a_bone = fk_pos_a[bone_a];
            Vec3 pos_b_bone = fk_pos_b[bone_b];

            // Body A columns: negative sign
            fill_rotation_columns(skeleton, map_a, fk_rots_a, fk_pos_a,
                                  bone_a, pos_a_bone, 0, row, w, -1.0f, J);

            // Body B columns: positive sign
            fill_rotation_columns(skeleton, map_b, fk_rots_b, fk_pos_b,
                                  bone_b, pos_b_bone, offset_b, row, w, 1.0f, J);

            // Root translation A: negative
            if (map_a.has_root_translation && is_ancestor_of(skeleton, map_a.root_bone, bone_a)) {
                u32 rt = map_a.root_trans_start;
                f32 rw = w * config.root_weight_a;
                J(row + 0, rt + 0) = -rw;
                J(row + 1, rt + 1) = -rw;
                J(row + 2, rt + 2) = -rw;
            }

            // Root translation B: positive
            if (map_b.has_root_translation && is_ancestor_of(skeleton, map_b.root_bone, bone_b)) {
                u32 rt = offset_b + map_b.root_trans_start;
                f32 rw = w * config.root_weight_b;
                J(row + 0, rt + 0) = rw;
                J(row + 1, rt + 1) = rw;
                J(row + 2, rt + 2) = rw;
            }

            row += 3;
        }

        // DLS solve: Δθ = J^T (JJ^T + λ²I)^{-1} residual
        DenseMat Jt = mat_transpose(J);
        DenseMat JJt = mat_mul(J, Jt);
        mat_add_scaled_identity(JJt, config.damping * config.damping);

        std::vector<f32> y = solve_linear(JJt, residual);
        std::vector<f32> delta_theta = mat_vec_mul(Jt, y);

        // Apply deltas to each body
        apply_deltas(skeleton, setup_a, map_a, delta_theta, 0, config.max_step, config.root_weight_a, pose_a);
        apply_deltas(skeleton, setup_b, map_b, delta_theta, offset_b, config.max_step, config.root_weight_b, pose_b);

        // Recompute FK for next iteration
        compute_fk(skeleton, pose_a, fk_world_a, fk_rots_a, fk_pos_a);
        compute_fk(skeleton, pose_b, fk_world_b, fk_rots_b, fk_pos_b);
    }

    return result;
}

} // namespace kaizen
