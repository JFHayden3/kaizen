#include "kaizen/core/ik.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace kaizen {

// --- Helpers ---

static Vec3 quat_rotate(const Quat& q, const Vec3& v) {
    // q * v * q^-1 via optimized formula
    Vec3 u(q.x, q.y, q.z);
    f32 s = q.w;
    return 2.0f * glm::dot(u, v) * u + (s * s - glm::dot(u, u)) * v +
           2.0f * s * glm::cross(u, v);
}

static Quat rotation_between(const Vec3& from, const Vec3& to) {
    Vec3 f = glm::normalize(from);
    Vec3 t = glm::normalize(to);
    f32 d = glm::dot(f, t);

    if (d >= 1.0f - 1e-6f) {
        return Quat{1.0f, 0.0f, 0.0f, 0.0f};
    }
    if (d <= -1.0f + 1e-6f) {
        Vec3 axis = glm::cross(Vec3{1.0f, 0.0f, 0.0f}, f);
        if (glm::dot(axis, axis) < 1e-6f) {
            axis = glm::cross(Vec3{0.0f, 1.0f, 0.0f}, f);
        }
        return glm::angleAxis(glm::pi<f32>(), glm::normalize(axis));
    }

    Vec3 axis = glm::cross(f, t);
    f32 w = 1.0f + d;
    return glm::normalize(Quat{w, axis.x, axis.y, axis.z});
}

// --- Joint constraint enforcement ---

static Quat apply_hinge_constraint(const Quat& q, const HingeLimit& limit) {
    // Extract twist angle around hinge axis
    Vec3 v(q.x, q.y, q.z);
    f32 p = glm::dot(v, limit.axis);
    f32 angle = 2.0f * std::atan2(p, q.w);

    // Normalize to [-pi, pi]
    if (angle > glm::pi<f32>()) angle -= 2.0f * glm::pi<f32>();
    if (angle < -glm::pi<f32>()) angle += 2.0f * glm::pi<f32>();

    angle = std::clamp(angle, limit.min_angle, limit.max_angle);
    return glm::angleAxis(angle, limit.axis);
}

static Quat apply_cone_constraint(const Quat& q, const ConeLimit& limit) {
    Vec3 rotated = quat_rotate(q, limit.axis);
    f32 d = std::clamp(glm::dot(rotated, limit.axis), -1.0f, 1.0f);
    f32 angle = std::acos(d);

    if (angle <= limit.half_angle) return q;

    Vec3 cross = glm::cross(limit.axis, rotated);
    if (glm::dot(cross, cross) < 1e-12f) {
        cross = glm::cross(limit.axis, Vec3{1.0f, 0.0f, 0.0f});
        if (glm::dot(cross, cross) < 1e-6f) {
            cross = glm::cross(limit.axis, Vec3{0.0f, 1.0f, 0.0f});
        }
    }
    Vec3 rot_axis = glm::normalize(cross);
    Vec3 clamped_dir =
        quat_rotate(glm::angleAxis(limit.half_angle, rot_axis), limit.axis);
    Quat correction = rotation_between(rotated, clamped_dir);
    return correction * q;
}

static Quat apply_constraint(const JointConstraint& constraint, const Quat& q) {
    if (auto* hinge = std::get_if<HingeLimit>(&constraint)) {
        return apply_hinge_constraint(q, *hinge);
    }
    if (auto* cone = std::get_if<ConeLimit>(&constraint)) {
        return apply_cone_constraint(q, *cone);
    }
    return q;
}

// --- Tree FABRIK internals ---

struct AffectedSet {
    BoneId root = INVALID_BONE;
    std::unordered_set<BoneId> bones;
    std::unordered_map<BoneId, std::vector<BoneId>> children; // affected children per bone
    std::unordered_map<BoneId, Vec3> target_positions;        // bone → target position
    std::unordered_map<BoneId, f32> target_weights;           // bone → target weight
};

static AffectedSet build_affected_set(const Skeleton& skeleton,
                                       const std::vector<IKTarget>& targets) {
    AffectedSet set;

    // Trace path from each target bone to root, marking all bones along the way
    for (const auto& t : targets) {
        if (!t.active || t.bone == INVALID_BONE || t.bone >= skeleton.bone_count())
            continue;

        set.target_positions[t.bone] = t.position;
        set.target_weights[t.bone] = t.weight;

        BoneId cur = t.bone;
        while (cur != INVALID_BONE) {
            set.bones.insert(cur);
            cur = skeleton.bones[cur].parent;
        }
    }

    if (set.bones.empty()) return set;

    // Find the root of the affected tree (bone with no affected parent)
    // This will be the topmost bone (SPINE_MID typically)
    for (BoneId b : set.bones) {
        BoneId parent = skeleton.bones[b].parent;
        if (parent == INVALID_BONE || set.bones.find(parent) == set.bones.end()) {
            // This bone has no affected parent — it's a root candidate
            // Pick the one closest to the actual skeleton root (lowest ID as a tiebreaker)
            if (set.root == INVALID_BONE || b < set.root) {
                set.root = b;
            }
        }
    }

    // Build affected children map
    for (BoneId b : set.bones) {
        if (b == set.root) continue;
        BoneId parent = skeleton.bones[b].parent;
        if (set.bones.find(parent) != set.bones.end()) {
            set.children[parent].push_back(b);
        }
    }

    return set;
}

static void forward_solve(BoneId bone, const Skeleton& skeleton,
                           const AffectedSet& affected,
                           std::vector<Vec3>& positions) {
    auto children_it = affected.children.find(bone);
    bool has_children = children_it != affected.children.end() &&
                        !children_it->second.empty();

    if (has_children) {
        // Solve children first (recursion toward leaves)
        for (BoneId child : children_it->second) {
            forward_solve(child, skeleton, affected, positions);
        }

        // Each child proposes a position for this bone
        Vec3 sum{0.0f};
        for (BoneId child : children_it->second) {
            f32 bone_len = glm::length(skeleton.bones[child].rest_offset);
            Vec3 dir = positions[bone] - positions[child];
            f32 len = glm::length(dir);
            if (len > 1e-8f) {
                dir /= len;
            } else {
                dir = Vec3{0.0f, 1.0f, 0.0f};
            }
            sum += positions[child] + dir * bone_len;
        }
        positions[bone] = sum / static_cast<f32>(children_it->second.size());
    }

    // If this bone has a target, override with target position
    auto target_it = affected.target_positions.find(bone);
    if (target_it != affected.target_positions.end()) {
        positions[bone] = target_it->second;
    }
}

static void backward_solve(BoneId bone, const Skeleton& skeleton,
                            const AffectedSet& affected,
                            std::vector<Vec3>& positions) {
    auto children_it = affected.children.find(bone);
    if (children_it == affected.children.end()) return;

    for (BoneId child : children_it->second) {
        f32 bone_len = glm::length(skeleton.bones[child].rest_offset);
        Vec3 dir = positions[child] - positions[bone];
        f32 len = glm::length(dir);
        if (len > 1e-8f) {
            dir /= len;
        } else {
            dir = Vec3{0.0f, -1.0f, 0.0f};
        }
        positions[child] = positions[bone] + dir * bone_len;
        backward_solve(child, skeleton, affected, positions);
    }
}

// Convert FABRIK positions to local rotations with constraint enforcement.
// Walks affected bones in BoneId order (parent before child guaranteed by skeleton layout).
// Updates positions in-place to reflect constrained result.
static void apply_rotations(const Skeleton& skeleton,
                             const AffectedSet& affected,
                             const std::vector<JointConstraint>& constraints,
                             std::vector<Vec3>& positions,
                             Pose& pose,
                             const std::vector<Mat4>& world_transforms) {
    // Process bones in BoneId order — guarantees parent is processed before child
    // Collect and sort affected bones
    std::vector<BoneId> sorted_bones(affected.bones.begin(), affected.bones.end());
    std::sort(sorted_bones.begin(), sorted_bones.end());

    // Track world rotations as we go (parent's world rotation needed for child)
    std::unordered_map<BoneId, Quat> world_rotations;

    for (BoneId bone_id : sorted_bones) {
        auto children_it = affected.children.find(bone_id);
        if (children_it == affected.children.end() || children_it->second.empty()) {
            // Leaf bone or no affected children — no rotation to compute
            // Still need to record world rotation for consistency
            BoneId parent = skeleton.bones[bone_id].parent;
            Quat parent_world_rot;
            if (parent == INVALID_BONE) {
                parent_world_rot = Quat{1.0f, 0.0f, 0.0f, 0.0f};
            } else {
                auto it = world_rotations.find(parent);
                if (it != world_rotations.end()) {
                    parent_world_rot = it->second;
                } else {
                    parent_world_rot = glm::quat_cast(world_transforms[parent]);
                }
            }
            world_rotations[bone_id] = parent_world_rot * pose.transforms[bone_id].rotation;
            continue;
        }

        // Pick the first affected child to compute rotation toward
        BoneId child_id = children_it->second[0];
        const auto& bone = skeleton.bones[bone_id];
        const auto& child = skeleton.bones[child_id];

        // Get parent world rotation
        Quat parent_world_rot;
        BoneId parent = bone.parent;
        if (parent == INVALID_BONE) {
            parent_world_rot = Quat{1.0f, 0.0f, 0.0f, 0.0f};
        } else {
            auto it = world_rotations.find(parent);
            if (it != world_rotations.end()) {
                parent_world_rot = it->second;
            } else {
                parent_world_rot = glm::quat_cast(world_transforms[parent]);
            }
        }

        Vec3 desired_world = positions[child_id] - positions[bone_id];
        Vec3 desired_local = quat_rotate(glm::inverse(parent_world_rot), desired_world);
        Vec3 rest_child_dir = quat_rotate(bone.rest_rotation, child.rest_offset);

        if (glm::dot(desired_local, desired_local) < 1e-12f ||
            glm::dot(rest_child_dir, rest_child_dir) < 1e-12f) {
            world_rotations[bone_id] = parent_world_rot * bone.rest_rotation;
            continue;
        }

        Quat delta = rotation_between(rest_child_dir, desired_local);
        Quat new_local_rot = delta * bone.rest_rotation;

        if (bone_id < static_cast<BoneId>(constraints.size())) {
            new_local_rot = apply_constraint(constraints[bone_id], new_local_rot);
        }

        pose.transforms[bone_id].rotation = new_local_rot;
        Quat new_world_rot = parent_world_rot * new_local_rot;
        world_rotations[bone_id] = new_world_rot;

        // Recompute all affected children's positions from constrained rotation
        for (BoneId c : children_it->second) {
            positions[c] = positions[bone_id] +
                           quat_rotate(new_world_rot, skeleton.bones[c].rest_offset);
        }
    }
}

// --- Public API ---

void ik_solve(const Skeleton& skeleton, const IKSetup& setup,
              const std::vector<IKTarget>& targets, Pose& pose,
              const std::vector<Mat4>& world_transforms,
              const IKSolverConfig& config) {
    AffectedSet affected = build_affected_set(skeleton, targets);
    if (affected.bones.empty() || affected.root == INVALID_BONE) return;

    // Extract world positions for all affected bones
    std::vector<Vec3> positions(skeleton.bone_count());
    for (BoneId b : affected.bones) {
        positions[b] = Vec3(world_transforms[b][3]);
    }

    // Store original rotations for weight blending
    std::vector<Quat> original_rots(skeleton.bone_count());
    for (BoneId b : affected.bones) {
        original_rots[b] = pose.transforms[b].rotation;
    }

    Vec3 original_root_pos = positions[affected.root];

    for (u32 iter = 0; iter < config.max_iterations; ++iter) {
        // Check convergence: all targets within tolerance
        bool converged = true;
        for (const auto& t : targets) {
            if (!t.active || t.bone == INVALID_BONE) continue;
            f32 err = glm::length(positions[t.bone] - t.position);
            if (err >= config.tolerance) {
                converged = false;
                break;
            }
        }
        if (converged) break;

        // Forward pass: leaves toward root
        forward_solve(affected.root, skeleton, affected, positions);

        // Root pin: lerp between pinned and where forward pass wants root
        if (config.root_mobility > 0.0f) {
            positions[affected.root] = glm::mix(original_root_pos, positions[affected.root], config.root_mobility);
        } else {
            positions[affected.root] = original_root_pos;
        }

        // Backward pass: root toward leaves
        backward_solve(affected.root, skeleton, affected, positions);

        // Convert to rotations with constraints; updates positions to match
        apply_rotations(skeleton, affected, setup.constraints, positions, pose,
                        world_transforms);
    }

    // Apply accumulated root displacement to pose translation
    Vec3 root_displacement = positions[affected.root] - original_root_pos;
    pose.transforms[affected.root].translation += root_displacement;

    // Weight blend: slerp original↔solved by max target weight on path
    // For simplicity, use the max weight of any target whose path includes this bone
    for (BoneId b : affected.bones) {
        f32 max_weight = 0.0f;
        for (const auto& t : targets) {
            if (!t.active || t.bone == INVALID_BONE) continue;
            // Check if bone b is on the path from target bone to root
            BoneId cur = t.bone;
            while (cur != INVALID_BONE) {
                if (cur == b) {
                    max_weight = std::max(max_weight, t.weight);
                    break;
                }
                cur = skeleton.bones[cur].parent;
            }
        }
        if (max_weight < 1.0f) {
            pose.transforms[b].rotation = glm::slerp(
                original_rots[b], pose.transforms[b].rotation, max_weight);
        }
    }
}

IKSetup IKSetup::build_humanoid() {
    IKSetup setup;

    setup.constraints.resize(HumanBone::COUNT);

    // Spine and pelvis cones: limit torso bend to ±30° per segment
    setup.constraints[HumanBone::SPINE_MID] =
        ConeLimit{{0.0f, 1.0f, 0.0f}, glm::radians(30.0f)};
    setup.constraints[HumanBone::SPINE_UPPER] =
        ConeLimit{{0.0f, 1.0f, 0.0f}, glm::radians(30.0f)};
    setup.constraints[HumanBone::PELVIS] =
        ConeLimit{{0.0f, -1.0f, 0.0f}, glm::radians(30.0f)};

    // Elbow hinges: flexion only, 0-150 degrees
    // Left forearm: +Y axis, positive rotation = flexion (hand forward)
    setup.constraints[HumanBone::FOREARM_L] =
        HingeLimit{{0.0f, 1.0f, 0.0f}, 0.0f, glm::radians(150.0f)};
    // Right forearm: -Y axis (mirrored)
    setup.constraints[HumanBone::FOREARM_R] =
        HingeLimit{{0.0f, -1.0f, 0.0f}, 0.0f, glm::radians(150.0f)};

    // Knee hinges: flexion only, 0-150 degrees
    // +X axis, negative rotation = flexion (foot backward)
    setup.constraints[HumanBone::LOWER_LEG_L] =
        HingeLimit{{1.0f, 0.0f, 0.0f}, glm::radians(-150.0f), 0.0f};
    setup.constraints[HumanBone::LOWER_LEG_R] =
        HingeLimit{{1.0f, 0.0f, 0.0f}, glm::radians(-150.0f), 0.0f};

    // Shoulder cones: ~120 deg half-angle
    setup.constraints[HumanBone::UPPER_ARM_L] =
        ConeLimit{{1.0f, 0.0f, 0.0f}, glm::radians(120.0f)};
    setup.constraints[HumanBone::UPPER_ARM_R] =
        ConeLimit{{-1.0f, 0.0f, 0.0f}, glm::radians(120.0f)};

    // Hip cones: ~90 deg half-angle
    setup.constraints[HumanBone::UPPER_LEG_L] =
        ConeLimit{{0.0f, -1.0f, 0.0f}, glm::radians(90.0f)};
    setup.constraints[HumanBone::UPPER_LEG_R] =
        ConeLimit{{0.0f, -1.0f, 0.0f}, glm::radians(90.0f)};

    return setup;
}

} // namespace kaizen
