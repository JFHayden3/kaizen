#pragma once

#include "kaizen/core/math.h"
#include "kaizen/core/types.h"

#include <optional>
#include <string>
#include <vector>

namespace kaizen {

using BoneId = u16;
constexpr BoneId INVALID_BONE = 0xFFFF;

struct Bone {
    std::string name;
    BoneId parent = INVALID_BONE;
    Vec3 rest_offset{0.0f};  // offset from parent in parent's local space
    Quat rest_rotation{1.0f, 0.0f, 0.0f, 0.0f}; // identity
};

struct BoneTransform {
    Quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
    Vec3 translation{0.0f};
};

struct Skeleton {
    std::vector<Bone> bones;

    BoneId bone_count() const { return static_cast<BoneId>(bones.size()); }
};

struct Pose {
    std::vector<BoneTransform> transforms;

    static Pose from_rest(const Skeleton& skeleton);
};

// Named bone indices for the humanoid skeleton
namespace HumanBone {
    constexpr BoneId SPINE_MID    = 0;
    constexpr BoneId SPINE_UPPER  = 1;
    constexpr BoneId PELVIS       = 2;
    constexpr BoneId NECK         = 3;
    constexpr BoneId HEAD         = 4;
    // Head contact points (children of HEAD, rigid on skull)
    constexpr BoneId HEAD_TOP     = 5;
    constexpr BoneId FOREHEAD     = 6;
    constexpr BoneId CHIN         = 7;
    constexpr BoneId JAW_L        = 8;
    constexpr BoneId JAW_R        = 9;
    constexpr BoneId HEAD_SIDE_L  = 10;
    constexpr BoneId HEAD_SIDE_R  = 11;
    constexpr BoneId HEAD_BACK    = 12;
    // Arms
    constexpr BoneId CLAVICLE_L   = 13;
    constexpr BoneId UPPER_ARM_L  = 14;
    constexpr BoneId FOREARM_L    = 15;
    constexpr BoneId HAND_L       = 16;
    constexpr BoneId CLAVICLE_R   = 17;
    constexpr BoneId UPPER_ARM_R  = 18;
    constexpr BoneId FOREARM_R    = 19;
    constexpr BoneId HAND_R       = 20;
    // Legs
    constexpr BoneId UPPER_LEG_L  = 21;
    constexpr BoneId LOWER_LEG_L  = 22;
    constexpr BoneId FOOT_L       = 23;
    constexpr BoneId UPPER_LEG_R  = 24;
    constexpr BoneId LOWER_LEG_R  = 25;
    constexpr BoneId FOOT_R       = 26;
    constexpr BoneId COUNT        = 27;

    // First/last head contact bone for iteration
    constexpr BoneId HEAD_CONTACT_FIRST = HEAD_TOP;
    constexpr BoneId HEAD_CONTACT_LAST  = HEAD_BACK;
} // namespace HumanBone

Skeleton build_humanoid_skeleton();

std::vector<Mat4> compute_world_transforms(const Skeleton& skeleton, const Pose& pose);

std::optional<BoneId> find_bone(const Skeleton& skeleton, const std::string& name);

} // namespace kaizen
