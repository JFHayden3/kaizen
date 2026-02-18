#include "kaizen/core/jacobian_ik.h"
#include "ik_internal.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

namespace kaizen {

using detail::quat_rotate;
using detail::apply_constraint;

// --- Dense matrix (row-major) ---

struct DenseMat {
    u32 rows = 0;
    u32 cols = 0;
    std::vector<f32> data;

    DenseMat() = default;
    DenseMat(u32 r, u32 c) : rows(r), cols(c), data(r * c, 0.0f) {}

    f32& operator()(u32 r, u32 c) { return data[r * cols + c]; }
    f32 operator()(u32 r, u32 c) const { return data[r * cols + c]; }

    void set_zero() { std::fill(data.begin(), data.end(), 0.0f); }
};

// C = A * B
static DenseMat mat_mul(const DenseMat& A, const DenseMat& B) {
    DenseMat C(A.rows, B.cols);
    for (u32 i = 0; i < A.rows; ++i) {
        for (u32 k = 0; k < A.cols; ++k) {
            f32 a = A(i, k);
            if (a == 0.0f) continue;
            for (u32 j = 0; j < B.cols; ++j) {
                C(i, j) += a * B(k, j);
            }
        }
    }
    return C;
}

// B = A^T
static DenseMat mat_transpose(const DenseMat& A) {
    DenseMat B(A.cols, A.rows);
    for (u32 i = 0; i < A.rows; ++i) {
        for (u32 j = 0; j < A.cols; ++j) {
            B(j, i) = A(i, j);
        }
    }
    return B;
}

// A += s * I (adds scalar to diagonal)
static void mat_add_scaled_identity(DenseMat& A, f32 s) {
    u32 n = std::min(A.rows, A.cols);
    for (u32 i = 0; i < n; ++i) {
        A(i, i) += s;
    }
}

// y = A * x
static std::vector<f32> mat_vec_mul(const DenseMat& A, const std::vector<f32>& x) {
    std::vector<f32> y(A.rows, 0.0f);
    for (u32 i = 0; i < A.rows; ++i) {
        for (u32 j = 0; j < A.cols; ++j) {
            y[i] += A(i, j) * x[j];
        }
    }
    return y;
}

// Solve Ax = b via Gaussian elimination with partial pivoting.
// A is modified in place. Returns x.
static std::vector<f32> solve_linear(DenseMat A, std::vector<f32> b) {
    u32 n = A.rows;
    for (u32 col = 0; col < n; ++col) {
        // Partial pivot
        u32 max_row = col;
        f32 max_val = std::abs(A(col, col));
        for (u32 row = col + 1; row < n; ++row) {
            f32 v = std::abs(A(row, col));
            if (v > max_val) {
                max_val = v;
                max_row = row;
            }
        }
        if (max_row != col) {
            for (u32 j = 0; j < n; ++j) {
                std::swap(A(col, j), A(max_row, j));
            }
            std::swap(b[col], b[max_row]);
        }

        f32 pivot = A(col, col);
        if (std::abs(pivot) < 1e-12f) continue;

        for (u32 row = col + 1; row < n; ++row) {
            f32 factor = A(row, col) / pivot;
            for (u32 j = col; j < n; ++j) {
                A(row, j) -= factor * A(col, j);
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    std::vector<f32> x(n, 0.0f);
    for (i32 i = static_cast<i32>(n) - 1; i >= 0; --i) {
        f32 sum = b[static_cast<u32>(i)];
        for (u32 j = static_cast<u32>(i) + 1; j < n; ++j) {
            sum -= A(static_cast<u32>(i), j) * x[j];
        }
        f32 diag = A(static_cast<u32>(i), static_cast<u32>(i));
        x[static_cast<u32>(i)] = (std::abs(diag) > 1e-12f) ? sum / diag : 0.0f;
    }
    return x;
}

// --- DOF mapping ---

struct DOFMap {
    std::vector<BoneId> bones;       // bones with rotation DOFs, sorted root-first
    std::vector<u32> bone_dof_start; // index into theta for bone's first DOF
    u32 total_dofs = 0;
    bool has_root_translation = false;
    u32 root_trans_start = 0;        // index into theta for root translation DOFs
    BoneId root_bone = INVALID_BONE;
};

static DOFMap build_dof_map(const Skeleton& skeleton,
                            const std::vector<IKTarget>& targets,
                            bool root_translates) {
    DOFMap map;
    std::unordered_set<BoneId> bone_set;

    // Trace from each active target to root, collecting all ancestor bones
    for (const auto& t : targets) {
        if (!t.active || t.bone == INVALID_BONE || t.bone >= skeleton.bone_count())
            continue;
        BoneId cur = t.bone;
        while (cur != INVALID_BONE) {
            bone_set.insert(cur);
            cur = skeleton.bones[cur].parent;
        }
    }

    if (bone_set.empty()) return map;

    // Sort by BoneId (guarantees parent before child)
    map.bones.assign(bone_set.begin(), bone_set.end());
    std::sort(map.bones.begin(), map.bones.end());

    // Find the root (first bone with no parent in the set)
    for (BoneId b : map.bones) {
        BoneId parent = skeleton.bones[b].parent;
        if (parent == INVALID_BONE || bone_set.find(parent) == bone_set.end()) {
            map.root_bone = b;
            break;
        }
    }

    // Assign 3 rotation DOFs per bone
    map.bone_dof_start.resize(map.bones.size());
    u32 dof_idx = 0;
    for (size_t i = 0; i < map.bones.size(); ++i) {
        map.bone_dof_start[i] = dof_idx;
        dof_idx += 3;
    }

    // Optionally add 3 root translation DOFs
    if (root_translates && map.root_bone != INVALID_BONE) {
        map.has_root_translation = true;
        map.root_trans_start = dof_idx;
        dof_idx += 3;
    }

    map.total_dofs = dof_idx;
    return map;
}

// Find the index of a bone in the DOF map, or -1
static i32 find_bone_in_map(const DOFMap& map, BoneId bone) {
    for (size_t i = 0; i < map.bones.size(); ++i) {
        if (map.bones[i] == bone) return static_cast<i32>(i);
    }
    return -1;
}

// --- FK from pose (recompute world transforms) ---

static void compute_fk(const Skeleton& skeleton, const Pose& pose,
                        std::vector<Mat4>& world,
                        std::vector<Quat>& world_rots,
                        std::vector<Vec3>& world_pos) {
    u32 n = skeleton.bone_count();
    world.resize(n);
    world_rots.resize(n);
    world_pos.resize(n);

    for (BoneId i = 0; i < n; ++i) {
        const auto& bone = skeleton.bones[i];
        const auto& bt = pose.transforms[i];

        // Must match compute_world_transforms: T(translation) * R(rotation)
        Mat4 local = glm::translate(Mat4{1.0f}, bt.translation) *
                     glm::mat4_cast(bt.rotation);

        if (bone.parent == INVALID_BONE) {
            world[i] = local;
        } else {
            world[i] = world[bone.parent] * local;
        }

        world_rots[i] = glm::quat_cast(world[i]);
        world_pos[i] = Vec3(world[i][3]);
    }
}

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
