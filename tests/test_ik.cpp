#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "kaizen/core/ik.h"

#include <cmath>

using namespace kaizen;
using Catch::Matchers::WithinAbs;

static Vec3 position_from_mat4(const Mat4& m) { return Vec3(m[3]); }

// --- 1. build_humanoid creates constraints for all bones ---

TEST_CASE("IKSetup::build_humanoid creates constraints", "[ik]") {
    auto setup = IKSetup::build_humanoid();
    REQUIRE(setup.constraints.size() == HumanBone::COUNT);
}

// --- 2. FABRIK solves hand to a reachable target (within 1cm) ---

TEST_CASE("FABRIK reaches a reachable target", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    Vec3 hand_pos = position_from_mat4(world[HumanBone::HAND_L]);
    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = hand_pos + Vec3{-0.1f, 0.0f, -0.1f},
         .weight = 1.0f,
         .active = true},
    };

    ik_solve(skel, setup, targets, pose, world);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_hand = position_from_mat4(solved[HumanBone::HAND_L]);
    f32 error = glm::length(solved_hand - targets[0].position);
    REQUIRE(error < 0.01f);
}

// --- 3. Unreachable target: hand reaches in the right direction ---

TEST_CASE("Unreachable target: chain extends toward target", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    Vec3 target_pos{5.0f, 1.5f, 0.0f};
    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = target_pos,
         .weight = 1.0f,
         .active = true},
    };

    ik_solve(skel, setup, targets, pose, world);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_hand = position_from_mat4(solved[HumanBone::HAND_L]);
    Vec3 chain_root = position_from_mat4(solved[HumanBone::SPINE_MID]);

    Vec3 to_target = glm::normalize(target_pos - chain_root);
    Vec3 to_hand = glm::normalize(solved_hand - chain_root);
    f32 dot = glm::dot(to_target, to_hand);
    REQUIRE(dot > 0.7f);
}

// --- 4. Bone lengths preserved after solve ---

TEST_CASE("Bone lengths preserved after IK solve", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    Vec3 hand_pos = position_from_mat4(world[HumanBone::HAND_L]);
    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = hand_pos + Vec3{0.0f, -0.15f, -0.1f},
         .active = true},
    };

    ik_solve(skel, setup, targets, pose, world);
    auto solved = compute_world_transforms(skel, pose);

    // Check bone lengths along the left arm path
    BoneId arm_bones[] = {HumanBone::SPINE_MID, HumanBone::SPINE_UPPER,
                          HumanBone::CLAVICLE_L, HumanBone::UPPER_ARM_L,
                          HumanBone::FOREARM_L, HumanBone::HAND_L};
    for (size_t i = 0; i < 5; ++i) {
        Vec3 p1 = position_from_mat4(solved[arm_bones[i]]);
        Vec3 p2 = position_from_mat4(solved[arm_bones[i + 1]]);
        f32 expected = glm::length(skel.bones[arm_bones[i + 1]].rest_offset);
        f32 actual = glm::length(p2 - p1);
        REQUIRE_THAT(static_cast<double>(actual),
                     WithinAbs(static_cast<double>(expected), 0.005));
    }
}

// --- 5. Inactive targets leave pose unchanged ---

TEST_CASE("Inactive targets leave pose unchanged", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);
    auto original = pose;

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L, .active = false},
    };

    ik_solve(skel, setup, targets, pose, world);

    for (BoneId i = 0; i < skel.bone_count(); ++i) {
        REQUIRE(pose.transforms[i].rotation == original.transforms[i].rotation);
    }
}

// --- 6. Target at current bone position is a no-op ---

TEST_CASE("Target at bone position is a no-op", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = position_from_mat4(world[HumanBone::HAND_L]),
         .active = true},
    };

    ik_solve(skel, setup, targets, pose, world);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_hand = position_from_mat4(solved[HumanBone::HAND_L]);
    REQUIRE(glm::length(solved_hand - targets[0].position) < 0.001f);
}

// --- 7. Multi-target simultaneous: both hands reach ---

TEST_CASE("Both hands reach targets simultaneously", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    Vec3 lh_pos = position_from_mat4(world[HumanBone::HAND_L]);
    Vec3 rh_pos = position_from_mat4(world[HumanBone::HAND_R]);

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = lh_pos + Vec3{-0.1f, 0.0f, -0.1f},
         .weight = 1.0f,
         .active = true},
        {.bone = HumanBone::HAND_R,
         .position = rh_pos + Vec3{0.1f, 0.0f, -0.1f},
         .weight = 1.0f,
         .active = true},
    };

    IKSolverConfig config;
    config.max_iterations = 30;
    ik_solve(skel, setup, targets, pose, world, config);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_lh = position_from_mat4(solved[HumanBone::HAND_L]);
    Vec3 solved_rh = position_from_mat4(solved[HumanBone::HAND_R]);

    // Slightly looser tolerance — shared spine bones must compromise
    REQUIRE(glm::length(solved_lh - targets[0].position) < 0.05f);
    REQUIRE(glm::length(solved_rh - targets[1].position) < 0.05f);
}

// --- 8. Non-terminal target: elbow positioning ---

TEST_CASE("Non-terminal target on forearm reaches", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    Vec3 forearm_pos = position_from_mat4(world[HumanBone::FOREARM_L]);
    Vec3 target_pos = forearm_pos + Vec3{0.0f, 0.0f, -0.15f};

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::FOREARM_L,
         .position = target_pos,
         .weight = 1.0f,
         .active = true},
    };

    IKSolverConfig config;
    config.max_iterations = 20;
    ik_solve(skel, setup, targets, pose, world, config);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_forearm = position_from_mat4(solved[HumanBone::FOREARM_L]);
    REQUIRE(glm::length(solved_forearm - target_pos) < 0.02f);
}

// --- 9. Mixed targets: hand + foot simultaneously ---

TEST_CASE("Hand and foot targets solved simultaneously", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    Vec3 hand_pos = position_from_mat4(world[HumanBone::HAND_L]);
    Vec3 foot_pos = position_from_mat4(world[HumanBone::FOOT_L]);

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = hand_pos + Vec3{-0.1f, 0.0f, -0.1f},
         .weight = 1.0f,
         .active = true},
        {.bone = HumanBone::FOOT_L,
         .position = foot_pos + Vec3{0.0f, 0.0f, -0.1f},
         .weight = 1.0f,
         .active = true},
    };

    IKSolverConfig config;
    config.max_iterations = 20;
    ik_solve(skel, setup, targets, pose, world, config);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_hand = position_from_mat4(solved[HumanBone::HAND_L]);
    Vec3 solved_foot = position_from_mat4(solved[HumanBone::FOOT_L]);

    // Both should get reasonably close (spine compromises; knee constraints
    // limit how far the foot can reach forward)
    REQUIRE(glm::length(solved_hand - targets[0].position) < 0.05f);
    REQUIRE(glm::length(solved_foot - targets[1].position) < 0.12f);
}

// --- 10. Elbow hinge prevents hyperextension ---

TEST_CASE("Elbow hinge constrains rotation to hinge axis", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = Vec3{0.2f, 1.3f, -0.3f},
         .active = true},
    };

    ik_solve(skel, setup, targets, pose, world);

    const auto* hinge =
        std::get_if<HingeLimit>(&setup.constraints[HumanBone::FOREARM_L]);
    REQUIRE(hinge != nullptr);

    Quat forearm_rot = pose.transforms[HumanBone::FOREARM_L].rotation;

    // Decompose: the rotation should be purely around the hinge axis
    Vec3 v(forearm_rot.x, forearm_rot.y, forearm_rot.z);
    f32 projection = glm::dot(v, hinge->axis);
    Vec3 swing = v - hinge->axis * projection;

    // Swing component should be near zero (pure hinge rotation)
    REQUIRE(glm::length(swing) < 0.02f);

    // Twist angle should be within [min_angle, max_angle]
    f32 twist = 2.0f * std::atan2(projection, forearm_rot.w);
    REQUIRE(twist >= hinge->min_angle - 0.01f);
    REQUIRE(twist <= hinge->max_angle + 0.01f);
}

// --- 11. Weight=0 leaves chain unchanged ---

TEST_CASE("Weight 0 leaves chain unchanged", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);
    auto original = pose;

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = Vec3{1.0f, 1.0f, 0.0f},
         .weight = 0.0f,
         .active = true},
    };

    ik_solve(skel, setup, targets, pose, world);

    // Check bones on the path from HAND_L to root
    BoneId path_bones[] = {HumanBone::SPINE_MID, HumanBone::SPINE_UPPER,
                           HumanBone::CLAVICLE_L, HumanBone::UPPER_ARM_L,
                           HumanBone::FOREARM_L};
    for (BoneId bone : path_bones) {
        f32 d = std::abs(
            glm::dot(pose.transforms[bone].rotation,
                     original.transforms[bone].rotation));
        REQUIRE(d > 0.999f);
    }
}

// --- 12. Spine leans when arm target is far ---

TEST_CASE("Spine leans toward far arm target", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);
    auto rest_pose = pose;

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = Vec3{0.8f, 1.3f, -0.5f},
         .weight = 1.0f,
         .active = true},
    };

    ik_solve(skel, setup, targets, pose, world);

    // At least one spine bone should have rotated away from rest
    Quat spine_mid_rest = rest_pose.transforms[HumanBone::SPINE_MID].rotation;
    Quat spine_mid_solved = pose.transforms[HumanBone::SPINE_MID].rotation;
    Quat spine_upper_rest = rest_pose.transforms[HumanBone::SPINE_UPPER].rotation;
    Quat spine_upper_solved = pose.transforms[HumanBone::SPINE_UPPER].rotation;

    f32 mid_dot = std::abs(glm::dot(spine_mid_rest, spine_mid_solved));
    f32 upper_dot = std::abs(glm::dot(spine_upper_rest, spine_upper_solved));

    // At least one spine bone must have moved noticeably (dot < 0.99 ≈ >8° rotation)
    bool spine_moved = (mid_dot < 0.99f) || (upper_dot < 0.99f);
    REQUIRE(spine_moved);
}

// --- 13. Empty targets vector is a no-op ---

TEST_CASE("Empty targets vector is a no-op", "[ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);
    auto original = pose;

    std::vector<IKTarget> targets;
    ik_solve(skel, setup, targets, pose, world);

    for (BoneId i = 0; i < skel.bone_count(); ++i) {
        REQUIRE(pose.transforms[i].rotation == original.transforms[i].rotation);
    }
}
