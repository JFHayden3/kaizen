#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "kaizen/core/skeleton.h"

#include <set>
#include <cmath>

using namespace kaizen;
using Catch::Matchers::WithinAbs;

static Vec3 position_from_mat4(const Mat4& m) {
    return Vec3(m[3]);
}

TEST_CASE("Humanoid skeleton has 27 bones", "[skeleton]") {
    auto skel = build_humanoid_skeleton();
    REQUIRE(skel.bone_count() == HumanBone::COUNT);
    REQUIRE(skel.bone_count() == 27);
}

TEST_CASE("Bones are in topological order", "[skeleton]") {
    auto skel = build_humanoid_skeleton();
    for (BoneId i = 0; i < skel.bone_count(); ++i) {
        if (skel.bones[i].parent != INVALID_BONE) {
            REQUIRE(skel.bones[i].parent < i);
        }
    }
}

TEST_CASE("All bone names are unique", "[skeleton]") {
    auto skel = build_humanoid_skeleton();
    std::set<std::string> names;
    for (const auto& bone : skel.bones) {
        REQUIRE(names.insert(bone.name).second);
    }
}

TEST_CASE("find_bone returns correct indices", "[skeleton]") {
    auto skel = build_humanoid_skeleton();

    auto pelvis = find_bone(skel, "pelvis");
    REQUIRE(pelvis.has_value());
    REQUIRE(*pelvis == HumanBone::PELVIS);

    auto hand_r = find_bone(skel, "hand_r");
    REQUIRE(hand_r.has_value());
    REQUIRE(*hand_r == HumanBone::HAND_R);

    auto missing = find_bone(skel, "nonexistent");
    REQUIRE_FALSE(missing.has_value());
}

TEST_CASE("Pose::from_rest has correct size", "[skeleton]") {
    auto skel = build_humanoid_skeleton();
    auto pose = Pose::from_rest(skel);
    REQUIRE(pose.transforms.size() == skel.bone_count());
}

TEST_CASE("FK rest pose produces expected joint positions", "[skeleton]") {
    auto skel = build_humanoid_skeleton();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    // Pelvis at (0, 0.95, 0)
    auto pelvis_pos = position_from_mat4(world[HumanBone::PELVIS]);
    REQUIRE_THAT(pelvis_pos.x, WithinAbs(0.0, 0.001));
    REQUIRE_THAT(pelvis_pos.y, WithinAbs(0.95, 0.001));

    // Head should be above pelvis
    auto head_pos = position_from_mat4(world[HumanBone::HEAD]);
    REQUIRE(head_pos.y > pelvis_pos.y);

    // Feet should be near ground (y ~ 0)
    auto foot_l = position_from_mat4(world[HumanBone::FOOT_L]);
    auto foot_r = position_from_mat4(world[HumanBone::FOOT_R]);
    REQUIRE_THAT(foot_l.y, WithinAbs(0.0, 0.01));
    REQUIRE_THAT(foot_r.y, WithinAbs(0.0, 0.01));

    // Hands should be to the sides
    auto hand_l = position_from_mat4(world[HumanBone::HAND_L]);
    auto hand_r = position_from_mat4(world[HumanBone::HAND_R]);
    REQUIRE(hand_l.x > 0.0f);
    REQUIRE(hand_r.x < 0.0f);
}

TEST_CASE("FK with rotated root propagates to children", "[skeleton]") {
    auto skel = build_humanoid_skeleton();
    auto pose = Pose::from_rest(skel);

    // Rotate root 90 degrees around Y axis
    pose.transforms[HumanBone::SPINE_MID].rotation =
        glm::angleAxis(glm::half_pi<float>(), Vec3{0.0f, 1.0f, 0.0f});

    auto world = compute_world_transforms(skel, pose);

    // Head should still be above pelvis
    auto pelvis_pos = position_from_mat4(world[HumanBone::PELVIS]);
    auto head_pos = position_from_mat4(world[HumanBone::HEAD]);
    REQUIRE(head_pos.y > pelvis_pos.y);

    // After 90 degree Y rotation (right-handed), left hand (which was at +X) maps to -Z
    auto hand_l = position_from_mat4(world[HumanBone::HAND_L]);
    REQUIRE_THAT(hand_l.x, WithinAbs(0.0, 0.05));
    REQUIRE(hand_l.z < -0.3f);
}

TEST_CASE("FK with rotated limb affects children", "[skeleton]") {
    auto skel = build_humanoid_skeleton();
    auto pose = Pose::from_rest(skel);

    auto world_rest = compute_world_transforms(skel, pose);
    auto forearm_rest = position_from_mat4(world_rest[HumanBone::FOREARM_L]);
    auto hand_rest = position_from_mat4(world_rest[HumanBone::HAND_L]);

    // Rotate left upper arm 90 degrees forward (around Z axis)
    pose.transforms[HumanBone::UPPER_ARM_L].rotation =
        glm::angleAxis(glm::half_pi<float>(), Vec3{0.0f, 0.0f, 1.0f}) *
        skel.bones[HumanBone::UPPER_ARM_L].rest_rotation;

    auto world_rotated = compute_world_transforms(skel, pose);
    auto forearm_rotated = position_from_mat4(world_rotated[HumanBone::FOREARM_L]);
    auto hand_rotated = position_from_mat4(world_rotated[HumanBone::HAND_L]);

    // Forearm and hand should have moved
    REQUIRE(glm::length(forearm_rotated - forearm_rest) > 0.1f);
    REQUIRE(glm::length(hand_rotated - hand_rest) > 0.1f);
}

TEST_CASE("Two poses with different root translations produce offset transforms", "[skeleton]") {
    auto skel = build_humanoid_skeleton();

    auto pose1 = Pose::from_rest(skel);
    pose1.transforms[HumanBone::SPINE_MID].translation = Vec3{-1.0f, 1.10f, 0.0f};

    auto pose2 = Pose::from_rest(skel);
    pose2.transforms[HumanBone::SPINE_MID].translation = Vec3{1.0f, 1.10f, 0.0f};

    auto world1 = compute_world_transforms(skel, pose1);
    auto world2 = compute_world_transforms(skel, pose2);

    // Every bone in pose2 should be 2 units to the right of pose1
    for (BoneId i = 0; i < skel.bone_count(); ++i) {
        auto p1 = position_from_mat4(world1[i]);
        auto p2 = position_from_mat4(world2[i]);
        REQUIRE_THAT(p2.x - p1.x, WithinAbs(2.0, 0.001));
        REQUIRE_THAT(p2.y - p1.y, WithinAbs(0.0, 0.001));
        REQUIRE_THAT(p2.z - p1.z, WithinAbs(0.0, 0.001));
    }
}
