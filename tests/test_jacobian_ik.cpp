#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "kaizen/core/jacobian_ik.h"

#include <cmath>

using namespace kaizen;
using Catch::Matchers::WithinAbs;

static Vec3 position_from_mat4(const Mat4& m) { return Vec3(m[3]); }

// --- 1. Single reachable target (hand moved 10cm) — error < 1cm ---

TEST_CASE("Jacobian IK reaches a reachable target", "[jacobian_ik]") {
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

    JacobianIKConfig config;
    config.root_translates = false;
    u32 iters = jacobian_ik_solve(skel, setup, targets, pose, world, config);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_hand = position_from_mat4(solved[HumanBone::HAND_L]);
    f32 error = glm::length(solved_hand - targets[0].position);
    REQUIRE(error < 0.01f);
    REQUIRE(iters > 0);
}

// --- 2. Unreachable target — chain extends toward it ---

TEST_CASE("Jacobian IK: unreachable target extends toward it", "[jacobian_ik]") {
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

    JacobianIKConfig config;
    config.root_translates = false;
    jacobian_ik_solve(skel, setup, targets, pose, world, config);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_hand = position_from_mat4(solved[HumanBone::HAND_L]);
    Vec3 chain_root = position_from_mat4(solved[HumanBone::SPINE_MID]);

    Vec3 to_target = glm::normalize(target_pos - chain_root);
    Vec3 to_hand = glm::normalize(solved_hand - chain_root);
    f32 dot = glm::dot(to_target, to_hand);
    REQUIRE(dot > 0.7f);
}

// --- 3. Bone lengths preserved after solve ---

TEST_CASE("Jacobian IK preserves bone lengths", "[jacobian_ik]") {
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

    JacobianIKConfig config;
    config.root_translates = false;
    jacobian_ik_solve(skel, setup, targets, pose, world, config);
    auto solved = compute_world_transforms(skel, pose);

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

// --- 4. Target at current position — no-op ---

TEST_CASE("Jacobian IK: target at bone position is a no-op", "[jacobian_ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = position_from_mat4(world[HumanBone::HAND_L]),
         .active = true},
    };

    u32 iters = jacobian_ik_solve(skel, setup, targets, pose, world);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_hand = position_from_mat4(solved[HumanBone::HAND_L]);
    REQUIRE(glm::length(solved_hand - targets[0].position) < 0.001f);
    REQUIRE(iters == 0);
}

// --- 5. Inactive targets — pose unchanged ---

TEST_CASE("Jacobian IK: inactive targets leave pose unchanged", "[jacobian_ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);
    auto original = pose;

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L, .active = false},
    };

    jacobian_ik_solve(skel, setup, targets, pose, world);

    for (BoneId i = 0; i < skel.bone_count(); ++i) {
        REQUIRE(pose.transforms[i].rotation == original.transforms[i].rotation);
    }
}

// --- 6. Empty targets — returns 0 iterations ---

TEST_CASE("Jacobian IK: empty targets returns 0", "[jacobian_ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    std::vector<IKTarget> targets;
    u32 iters = jacobian_ik_solve(skel, setup, targets, pose, world);
    REQUIRE(iters == 0);
}

// --- 7. Hinge constraint enforced (elbow stays on axis) ---

TEST_CASE("Jacobian IK enforces elbow hinge constraint", "[jacobian_ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = Vec3{0.2f, 1.3f, -0.3f},
         .active = true},
    };

    JacobianIKConfig config;
    config.root_translates = false;
    jacobian_ik_solve(skel, setup, targets, pose, world, config);

    const auto* hinge =
        std::get_if<HingeLimit>(&setup.constraints[HumanBone::FOREARM_L]);
    REQUIRE(hinge != nullptr);

    Quat forearm_rot = pose.transforms[HumanBone::FOREARM_L].rotation;
    Vec3 v(forearm_rot.x, forearm_rot.y, forearm_rot.z);
    f32 projection = glm::dot(v, hinge->axis);
    Vec3 swing = v - hinge->axis * projection;

    REQUIRE(glm::length(swing) < 0.02f);

    f32 twist = 2.0f * std::atan2(projection, forearm_rot.w);
    REQUIRE(twist >= hinge->min_angle - 0.01f);
    REQUIRE(twist <= hinge->max_angle + 0.01f);
}

// --- 8. Multi-target (both hands) — both within 5cm ---

TEST_CASE("Jacobian IK solves both hands simultaneously", "[jacobian_ik]") {
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

    JacobianIKConfig config;
    config.max_iterations = 40;
    config.root_translates = false;
    jacobian_ik_solve(skel, setup, targets, pose, world, config);
    auto solved = compute_world_transforms(skel, pose);

    Vec3 solved_lh = position_from_mat4(solved[HumanBone::HAND_L]);
    Vec3 solved_rh = position_from_mat4(solved[HumanBone::HAND_R]);

    REQUIRE(glm::length(solved_lh - targets[0].position) < 0.05f);
    REQUIRE(glm::length(solved_rh - targets[1].position) < 0.05f);
}

// --- 9. Root translation — root moves when enabled, stays fixed when disabled ---

TEST_CASE("Jacobian IK root translation", "[jacobian_ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();

    // Far target that requires root to move to reach
    Vec3 far_target{2.0f, 1.5f, 0.0f};
    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = far_target,
         .weight = 1.0f,
         .active = true},
    };

    // With root translation
    {
        auto pose = Pose::from_rest(skel);
        auto world = compute_world_transforms(skel, pose);
        Vec3 original_root = pose.transforms[HumanBone::WORLD_ROOT].translation;

        JacobianIKConfig config;
        config.root_translates = true;
        config.max_iterations = 40;
        jacobian_ik_solve(skel, setup, targets, pose, world, config);

        Vec3 root_delta = pose.transforms[HumanBone::WORLD_ROOT].translation - original_root;
        REQUIRE(glm::length(root_delta) > 0.01f);
    }

    // Without root translation
    {
        auto pose = Pose::from_rest(skel);
        auto world = compute_world_transforms(skel, pose);
        Vec3 original_root = pose.transforms[HumanBone::WORLD_ROOT].translation;

        JacobianIKConfig config;
        config.root_translates = false;
        config.max_iterations = 40;
        jacobian_ik_solve(skel, setup, targets, pose, world, config);

        Vec3 root_delta = pose.transforms[HumanBone::WORLD_ROOT].translation - original_root;
        REQUIRE(glm::length(root_delta) < 0.001f);
    }
}

// --- 10. Twist test — verify shoulder produces non-trivial axial rotation ---

TEST_CASE("Jacobian IK produces twist rotation", "[jacobian_ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    // Target hand across body and behind — requires significant shoulder internal rotation (twist).
    // The left arm extends in +X from rest. Moving it to the opposite side and behind
    // the body forces the upper arm to twist around its length axis.
    Vec3 cross_body_target = Vec3{-0.1f, 1.0f, 0.2f};

    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = cross_body_target,
         .weight = 1.0f,
         .active = true},
    };

    Quat rest_upper_arm = pose.transforms[HumanBone::UPPER_ARM_L].rotation;

    JacobianIKConfig config;
    config.root_translates = false;
    config.max_iterations = 40;
    jacobian_ik_solve(skel, setup, targets, pose, world, config);

    Quat solved_upper_arm = pose.transforms[HumanBone::UPPER_ARM_L].rotation;

    // The upper arm rotation should differ significantly from rest
    f32 dot = std::abs(glm::dot(rest_upper_arm, solved_upper_arm));
    REQUIRE(dot < 0.99f);

    // Decompose into swing-twist around the arm's primary axis (rest_offset direction)
    Vec3 arm_axis = glm::normalize(skel.bones[HumanBone::UPPER_ARM_L].rest_offset);
    Quat rel = solved_upper_arm * glm::inverse(rest_upper_arm);
    Vec3 rel_v(rel.x, rel.y, rel.z);
    f32 twist_proj = glm::dot(rel_v, arm_axis);

    // Non-trivial twist component means the solver exercised the axial DOF
    // (FABRIK's rotation_between can only produce swing, so twist_proj ~ 0 there)
    REQUIRE(std::abs(twist_proj) > 0.01f);
}

// --- 11. Temporal coherence — second call converges faster ---

TEST_CASE("Jacobian IK temporal coherence", "[jacobian_ik]") {
    auto skel = build_humanoid_skeleton();
    auto setup = IKSetup::build_humanoid();
    auto pose = Pose::from_rest(skel);
    auto world = compute_world_transforms(skel, pose);

    Vec3 hand_pos = position_from_mat4(world[HumanBone::HAND_L]);
    std::vector<IKTarget> targets = {
        {.bone = HumanBone::HAND_L,
         .position = hand_pos + Vec3{-0.1f, 0.05f, -0.1f},
         .weight = 1.0f,
         .active = true},
    };

    JacobianIKConfig config;
    config.root_translates = false;

    // First solve from rest
    u32 iters1 = jacobian_ik_solve(skel, setup, targets, pose, world, config);

    // Second solve with same target but starting from previous solution
    world = compute_world_transforms(skel, pose);
    u32 iters2 = jacobian_ik_solve(skel, setup, targets, pose, world, config);

    REQUIRE(iters2 <= iters1);
}
