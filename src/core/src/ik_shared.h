#pragma once

#include "kaizen/core/ik.h"

#include <algorithm>
#include <unordered_set>
#include <vector>

namespace kaizen {

struct DOFMap {
    std::vector<BoneId> bones;       // bones with rotation DOFs, sorted root-first
    std::vector<u32> bone_dof_start; // index into theta for bone's first DOF
    u32 total_dofs = 0;
    bool has_root_translation = false;
    u32 root_trans_start = 0;        // index into theta for root translation DOFs
    BoneId root_bone = INVALID_BONE;
};

inline DOFMap build_dof_map(const Skeleton& skeleton,
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
inline i32 find_bone_in_map(const DOFMap& map, BoneId bone) {
    for (size_t i = 0; i < map.bones.size(); ++i) {
        if (map.bones[i] == bone) return static_cast<i32>(i);
    }
    return -1;
}

// Recompute world transforms from pose
inline void compute_fk(const Skeleton& skeleton, const Pose& pose,
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

} // namespace kaizen
