#include "kaizen/core/skeleton.h"

#include <glm/gtc/matrix_transform.hpp>

namespace kaizen {

Pose Pose::from_rest(const Skeleton& skeleton) {
    Pose pose;
    pose.transforms.resize(skeleton.bone_count());
    for (BoneId i = 0; i < skeleton.bone_count(); ++i) {
        pose.transforms[i].rotation = skeleton.bones[i].rest_rotation;
        pose.transforms[i].translation = skeleton.bones[i].rest_offset;
    }
    return pose;
}

Skeleton build_humanoid_skeleton() {
    Skeleton skel;
    skel.bones.resize(HumanBone::COUNT);

    auto& b = skel.bones;

    // Proportions for ~1.75m character
    // Y is up, character faces -Z in rest pose

    // WORLD_ROOT: unconstrained whole-body position and orientation
    b[HumanBone::WORLD_ROOT]  = {"world_root",  INVALID_BONE,          {0.0f, 1.10f, 0.0f}};

    // Spine — SPINE_MID is the hub; arms chain up through SPINE_UPPER, legs chain down through PELVIS
    b[HumanBone::SPINE_MID]   = {"spine_mid",   HumanBone::WORLD_ROOT, {0.0f, 0.0f, 0.0f}};
    b[HumanBone::SPINE_UPPER] = {"spine_upper",  HumanBone::SPINE_MID,  {0.0f, 0.20f, 0.0f}};
    b[HumanBone::PELVIS]      = {"pelvis",       HumanBone::SPINE_MID,  {0.0f, -0.15f, 0.0f}};

    // Head
    b[HumanBone::NECK]        = {"neck",         HumanBone::SPINE_UPPER, {0.0f, 0.12f, 0.0f}};
    b[HumanBone::HEAD]        = {"head",         HumanBone::NECK,        {0.0f, 0.10f, 0.0f}};

    // Head contact points — fixed offsets from HEAD, character faces -Z
    b[HumanBone::HEAD_TOP]    = {"head_top",     HumanBone::HEAD, {0.0f, 0.17f, 0.0f}};
    b[HumanBone::FOREHEAD]    = {"forehead",     HumanBone::HEAD, {0.0f, 0.13f, -0.07f}};
    b[HumanBone::CHIN]        = {"chin",         HumanBone::HEAD, {0.0f, 0.01f, -0.07f}};
    b[HumanBone::JAW_L]       = {"jaw_l",        HumanBone::HEAD, {0.06f, 0.03f, -0.05f}};
    b[HumanBone::JAW_R]       = {"jaw_r",        HumanBone::HEAD, {-0.06f, 0.03f, -0.05f}};
    b[HumanBone::HEAD_SIDE_L] = {"head_side_l",  HumanBone::HEAD, {0.09f, 0.08f, 0.0f}};
    b[HumanBone::HEAD_SIDE_R] = {"head_side_r",  HumanBone::HEAD, {-0.09f, 0.08f, 0.0f}};
    b[HumanBone::HEAD_BACK]   = {"head_back",    HumanBone::HEAD, {0.0f, 0.10f, 0.08f}};

    // Left arm
    b[HumanBone::CLAVICLE_L]  = {"clavicle_l",  HumanBone::SPINE_UPPER, {0.08f, 0.10f, 0.0f}};
    b[HumanBone::UPPER_ARM_L] = {"upper_arm_l", HumanBone::CLAVICLE_L,  {0.12f, 0.0f, 0.0f}};
    b[HumanBone::FOREARM_L]   = {"forearm_l",   HumanBone::UPPER_ARM_L, {0.27f, 0.0f, 0.0f}};
    b[HumanBone::HAND_L]      = {"hand_l",      HumanBone::FOREARM_L,   {0.25f, 0.0f, 0.0f}};

    // Right arm
    b[HumanBone::CLAVICLE_R]  = {"clavicle_r",  HumanBone::SPINE_UPPER, {-0.08f, 0.10f, 0.0f}};
    b[HumanBone::UPPER_ARM_R] = {"upper_arm_r", HumanBone::CLAVICLE_R,  {-0.12f, 0.0f, 0.0f}};
    b[HumanBone::FOREARM_R]   = {"forearm_r",   HumanBone::UPPER_ARM_R, {-0.27f, 0.0f, 0.0f}};
    b[HumanBone::HAND_R]      = {"hand_r",      HumanBone::FOREARM_R,   {-0.25f, 0.0f, 0.0f}};

    // Left leg
    b[HumanBone::UPPER_LEG_L] = {"upper_leg_l", HumanBone::PELVIS,      {0.10f, -0.05f, 0.0f}};
    b[HumanBone::LOWER_LEG_L] = {"lower_leg_l", HumanBone::UPPER_LEG_L, {0.0f, -0.45f, 0.0f}};
    b[HumanBone::FOOT_L]      = {"foot_l",      HumanBone::LOWER_LEG_L, {0.0f, -0.45f, 0.0f}};

    // Right leg
    b[HumanBone::UPPER_LEG_R] = {"upper_leg_r", HumanBone::PELVIS,      {-0.10f, -0.05f, 0.0f}};
    b[HumanBone::LOWER_LEG_R] = {"lower_leg_r", HumanBone::UPPER_LEG_R, {0.0f, -0.45f, 0.0f}};
    b[HumanBone::FOOT_R]      = {"foot_r",      HumanBone::LOWER_LEG_R, {0.0f, -0.45f, 0.0f}};

    // Hands (extend from wrist) and feet (extend from ankle)
    b[HumanBone::FINGERS_L]   = {"fingers_l",   HumanBone::HAND_L,      {0.10f, 0.0f, 0.0f}};
    b[HumanBone::FINGERS_R]   = {"fingers_r",   HumanBone::HAND_R,      {-0.10f, 0.0f, 0.0f}};
    b[HumanBone::TOES_L]      = {"toes_l",      HumanBone::FOOT_L,      {0.0f, 0.0f, -0.18f}};
    b[HumanBone::TOES_R]      = {"toes_r",      HumanBone::FOOT_R,      {0.0f, 0.0f, -0.18f}};

    return skel;
}

std::vector<Mat4> compute_world_transforms(const Skeleton& skeleton, const Pose& pose) {
    const auto count = skeleton.bone_count();
    std::vector<Mat4> world(count, Mat4{1.0f});

    for (BoneId i = 0; i < count; ++i) {
        const auto& bone = skeleton.bones[i];
        const auto& xform = pose.transforms[i];

        // Build local transform: translate by offset, then rotate
        Mat4 local = glm::translate(Mat4{1.0f}, xform.translation) *
                     glm::mat4_cast(xform.rotation);

        if (bone.parent == INVALID_BONE) {
            world[i] = local;
        } else {
            world[i] = world[bone.parent] * local;
        }
    }

    return world;
}

std::optional<BoneId> find_bone(const Skeleton& skeleton, const std::string& name) {
    for (BoneId i = 0; i < skeleton.bone_count(); ++i) {
        if (skeleton.bones[i].name == name) {
            return i;
        }
    }
    return std::nullopt;
}

} // namespace kaizen
