#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "kaizen/core/two_body_ik.h"

#include <cmath>

using namespace kaizen;
using Catch::Matchers::WithinAbs;

static Vec3 bone_pos(const std::vector<Mat4>& world, BoneId bone) {
    return Vec3(world[bone][3]);
}

// Helper: set up two bodies facing each other
struct TwoBodySetup {
    Skeleton skeleton;
    IKSetup setup_a, setup_b;
    Pose pose_a, pose_b;
    std::vector<Mat4> world_a, world_b;

    TwoBodySetup() {
        skeleton = build_humanoid_skeleton();
        setup_a = IKSetup::build_humanoid();
        setup_b = IKSetup::build_humanoid();
        pose_a = Pose::from_rest(skeleton);
        pose_b = Pose::from_rest(skeleton);

        // Place A and B facing each other
        pose_a.transforms[HumanBone::WORLD_ROOT].translation = Vec3{-0.8f, 1.10f, 0.0f};
        pose_b.transforms[HumanBone::WORLD_ROOT].translation = Vec3{0.8f, 1.10f, 0.0f};
        pose_b.transforms[HumanBone::WORLD_ROOT].rotation =
            glm::angleAxis(glm::pi<f32>(), Vec3{0.0f, 1.0f, 0.0f});

        world_a = compute_world_transforms(skeleton, pose_a);
        world_b = compute_world_transforms(skeleton, pose_b);
    }
};

// --- 1. Contact pulls bodies together ---

TEST_CASE("Two-body IK: contact pulls bodies together", "[two_body_ik]") {
    TwoBodySetup s;

    std::vector<ContactConstraint> contacts = {
        {.bone_a = HumanBone::HAND_L, .bone_b = HumanBone::FOREARM_R, .weight = 1.0f, .active = true},
    };

    Vec3 hand_a_before = bone_pos(s.world_a, HumanBone::HAND_L);
    Vec3 forearm_b_before = bone_pos(s.world_b, HumanBone::FOREARM_R);
    f32 dist_before = glm::length(hand_a_before - forearm_b_before);

    TwoBodyIKConfig config;
    config.max_iterations = 40;
    auto result = two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                                    {}, {}, contacts,
                                    s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    auto solved_a = compute_world_transforms(s.skeleton, s.pose_a);
    auto solved_b = compute_world_transforms(s.skeleton, s.pose_b);

    Vec3 hand_a_after = bone_pos(solved_a, HumanBone::HAND_L);
    Vec3 forearm_b_after = bone_pos(solved_b, HumanBone::FOREARM_R);
    f32 dist_after = glm::length(hand_a_after - forearm_b_after);

    REQUIRE(dist_after < dist_before);
    REQUIRE(result.iterations > 0);
}

// --- 2. Root weight asymmetry ---

TEST_CASE("Two-body IK: root weight asymmetry", "[two_body_ik]") {
    TwoBodySetup s;

    std::vector<ContactConstraint> contacts = {
        {.bone_a = HumanBone::HAND_L, .bone_b = HumanBone::FOREARM_R, .weight = 1.0f, .active = true},
    };

    Vec3 root_a_before = s.pose_a.transforms[HumanBone::WORLD_ROOT].translation;
    Vec3 root_b_before = s.pose_b.transforms[HumanBone::WORLD_ROOT].translation;

    TwoBodyIKConfig config;
    config.max_iterations = 40;
    config.root_weight_a = 0.0f;  // A's root doesn't translate
    config.root_weight_b = 1.0f;  // B's root translates freely

    two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                      {}, {}, contacts,
                      s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    Vec3 root_a_delta = s.pose_a.transforms[HumanBone::WORLD_ROOT].translation - root_a_before;
    Vec3 root_b_delta = s.pose_b.transforms[HumanBone::WORLD_ROOT].translation - root_b_before;

    // B should move more than A (A's root weight is 0, but A can still rotate)
    // The key check: A's root barely moves, B's root moves significantly
    REQUIRE(glm::length(root_a_delta) < 0.01f);
    REQUIRE(glm::length(root_b_delta) > glm::length(root_a_delta));
}

// --- 3. Per-body world targets — targets on A only, B unchanged ---

TEST_CASE("Two-body IK: per-body world targets", "[two_body_ik]") {
    TwoBodySetup s;

    Vec3 hand_a_pos = bone_pos(s.world_a, HumanBone::HAND_L);
    std::vector<IKTarget> targets_a = {
        {.bone = HumanBone::HAND_L,
         .position = hand_a_pos + Vec3{-0.1f, 0.0f, -0.1f},
         .weight = 1.0f,
         .active = true},
    };

    Pose pose_b_copy = s.pose_b;

    TwoBodyIKConfig config;
    config.root_translates_a = false;
    config.root_translates_b = false;
    two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                      targets_a, {}, {},
                      s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    // B should be unchanged (no targets, no contacts)
    for (BoneId i = 0; i < s.skeleton.bone_count(); ++i) {
        REQUIRE(s.pose_b.transforms[i].rotation == pose_b_copy.transforms[i].rotation);
        REQUIRE(s.pose_b.transforms[i].translation == pose_b_copy.transforms[i].translation);
    }

    // A should have moved toward target
    auto solved_a = compute_world_transforms(s.skeleton, s.pose_a);
    Vec3 solved_hand = bone_pos(solved_a, HumanBone::HAND_L);
    f32 error = glm::length(solved_hand - targets_a[0].position);
    REQUIRE(error < 0.05f);
}

// --- 4. Mixed contacts + world targets ---

TEST_CASE("Two-body IK: mixed contacts and world targets", "[two_body_ik]") {
    TwoBodySetup s;

    Vec3 hand_a_pos = bone_pos(s.world_a, HumanBone::HAND_R);
    std::vector<IKTarget> targets_a = {
        {.bone = HumanBone::HAND_R,
         .position = hand_a_pos + Vec3{0.1f, 0.0f, -0.1f},
         .weight = 1.0f,
         .active = true},
    };

    std::vector<ContactConstraint> contacts = {
        {.bone_a = HumanBone::HAND_L, .bone_b = HumanBone::UPPER_ARM_R, .weight = 1.0f, .active = true},
    };

    Vec3 contact_a_before = bone_pos(s.world_a, HumanBone::HAND_L);
    Vec3 contact_b_before = bone_pos(s.world_b, HumanBone::UPPER_ARM_R);
    f32 contact_dist_before = glm::length(contact_a_before - contact_b_before);

    TwoBodyIKConfig config;
    config.max_iterations = 40;
    two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                      targets_a, {}, contacts,
                      s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    auto solved_a = compute_world_transforms(s.skeleton, s.pose_a);
    auto solved_b = compute_world_transforms(s.skeleton, s.pose_b);

    // World target should improve
    Vec3 solved_hand = bone_pos(solved_a, HumanBone::HAND_R);
    f32 target_error = glm::length(solved_hand - targets_a[0].position);
    REQUIRE(target_error < 0.1f);

    // Contact distance should decrease
    Vec3 contact_a_after = bone_pos(solved_a, HumanBone::HAND_L);
    Vec3 contact_b_after = bone_pos(solved_b, HumanBone::UPPER_ARM_R);
    f32 contact_dist_after = glm::length(contact_a_after - contact_b_after);
    REQUIRE(contact_dist_after < contact_dist_before);
}

// --- 5. Pinned roots — only rotations adjust ---

TEST_CASE("Two-body IK: pinned roots", "[two_body_ik]") {
    TwoBodySetup s;

    std::vector<ContactConstraint> contacts = {
        {.bone_a = HumanBone::HAND_L, .bone_b = HumanBone::FOREARM_R, .weight = 1.0f, .active = true},
    };

    Vec3 root_a_before = s.pose_a.transforms[HumanBone::WORLD_ROOT].translation;
    Vec3 root_b_before = s.pose_b.transforms[HumanBone::WORLD_ROOT].translation;

    TwoBodyIKConfig config;
    config.max_iterations = 30;
    config.root_translates_a = false;
    config.root_translates_b = false;
    two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                      {}, {}, contacts,
                      s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    Vec3 root_a_after = s.pose_a.transforms[HumanBone::WORLD_ROOT].translation;
    Vec3 root_b_after = s.pose_b.transforms[HumanBone::WORLD_ROOT].translation;

    REQUIRE(glm::length(root_a_after - root_a_before) < 0.001f);
    REQUIRE(glm::length(root_b_after - root_b_before) < 0.001f);
}

// --- 6. Bone lengths preserved ---

TEST_CASE("Two-body IK: bone lengths preserved", "[two_body_ik]") {
    TwoBodySetup s;

    std::vector<ContactConstraint> contacts = {
        {.bone_a = HumanBone::HAND_L, .bone_b = HumanBone::FOREARM_R, .weight = 1.0f, .active = true},
    };

    TwoBodyIKConfig config;
    config.max_iterations = 30;
    two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                      {}, {}, contacts,
                      s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    auto solved_a = compute_world_transforms(s.skeleton, s.pose_a);
    auto solved_b = compute_world_transforms(s.skeleton, s.pose_b);

    BoneId arm_chain[] = {HumanBone::SPINE_MID, HumanBone::SPINE_UPPER,
                          HumanBone::CLAVICLE_L, HumanBone::UPPER_ARM_L,
                          HumanBone::FOREARM_L, HumanBone::HAND_L};

    for (const auto* world : {&solved_a, &solved_b}) {
        for (size_t i = 0; i < 5; ++i) {
            Vec3 p1 = bone_pos(*world, arm_chain[i]);
            Vec3 p2 = bone_pos(*world, arm_chain[i + 1]);
            f32 expected = glm::length(s.skeleton.bones[arm_chain[i + 1]].rest_offset);
            f32 actual = glm::length(p2 - p1);
            REQUIRE_THAT(static_cast<double>(actual),
                         WithinAbs(static_cast<double>(expected), 0.005));
        }
    }
}

// --- 7. Inactive contacts ignored ---

TEST_CASE("Two-body IK: inactive contacts ignored", "[two_body_ik]") {
    TwoBodySetup s;

    Pose pose_a_copy = s.pose_a;
    Pose pose_b_copy = s.pose_b;

    std::vector<ContactConstraint> contacts = {
        {.bone_a = HumanBone::HAND_L, .bone_b = HumanBone::FOREARM_R, .weight = 1.0f, .active = false},
        {.bone_a = HumanBone::HAND_R, .bone_b = HumanBone::UPPER_ARM_L, .weight = 1.0f, .active = false},
    };

    auto result = two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                                    {}, {}, contacts,
                                    s.pose_a, s.pose_b, s.world_a, s.world_b);

    // No active constraints means no work
    REQUIRE(result.iterations == 0);

    for (BoneId i = 0; i < s.skeleton.bone_count(); ++i) {
        REQUIRE(s.pose_a.transforms[i].rotation == pose_a_copy.transforms[i].rotation);
        REQUIRE(s.pose_b.transforms[i].rotation == pose_b_copy.transforms[i].rotation);
    }
}

// --- 8. Empty inputs ---

TEST_CASE("Two-body IK: empty inputs returns 0 iterations", "[two_body_ik]") {
    TwoBodySetup s;

    auto result = two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                                    {}, {}, {},
                                    s.pose_a, s.pose_b, s.world_a, s.world_b);

    REQUIRE(result.iterations == 0);
}

// --- 9. Symmetric contact — both bodies move approximately equally ---

TEST_CASE("Two-body IK: symmetric contact", "[two_body_ik]") {
    TwoBodySetup s;

    // Use hand-to-hand contact — symmetric bones
    std::vector<ContactConstraint> contacts = {
        {.bone_a = HumanBone::HAND_L, .bone_b = HumanBone::HAND_R, .weight = 1.0f, .active = true},
    };

    Vec3 root_a_before = s.pose_a.transforms[HumanBone::WORLD_ROOT].translation;
    Vec3 root_b_before = s.pose_b.transforms[HumanBone::WORLD_ROOT].translation;

    TwoBodyIKConfig config;
    config.max_iterations = 40;
    config.root_weight_a = 0.3f;
    config.root_weight_b = 0.3f;
    two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                      {}, {}, contacts,
                      s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    Vec3 root_a_delta = s.pose_a.transforms[HumanBone::WORLD_ROOT].translation - root_a_before;
    Vec3 root_b_delta = s.pose_b.transforms[HumanBone::WORLD_ROOT].translation - root_b_before;

    // With equal weights and mirrored setup, both roots should move roughly equally
    f32 ratio = glm::length(root_a_delta) / std::max(glm::length(root_b_delta), 0.001f);
    REQUIRE(ratio > 0.3f);
    REQUIRE(ratio < 3.0f);
}

// --- 10. Temporal coherence — second solve converges faster ---

TEST_CASE("Two-body IK: temporal coherence", "[two_body_ik]") {
    TwoBodySetup s;

    std::vector<ContactConstraint> contacts = {
        {.bone_a = HumanBone::HAND_L, .bone_b = HumanBone::FOREARM_R, .weight = 1.0f, .active = true},
    };

    TwoBodyIKConfig config;
    config.max_iterations = 40;

    // First solve from rest
    auto result1 = two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                                     {}, {}, contacts,
                                     s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    // Second solve from warm start
    s.world_a = compute_world_transforms(s.skeleton, s.pose_a);
    s.world_b = compute_world_transforms(s.skeleton, s.pose_b);
    auto result2 = two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                                     {}, {}, contacts,
                                     s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    REQUIRE(result2.iterations <= result1.iterations);
}

// --- 11. Multiple contacts — all improve ---

TEST_CASE("Two-body IK: multiple contacts all improve", "[two_body_ik]") {
    TwoBodySetup s;

    std::vector<ContactConstraint> contacts = {
        {.bone_a = HumanBone::HAND_L, .bone_b = HumanBone::FOREARM_R, .weight = 1.0f, .active = true},
        {.bone_a = HumanBone::HAND_R, .bone_b = HumanBone::UPPER_ARM_L, .weight = 1.0f, .active = true},
        {.bone_a = HumanBone::FOOT_L, .bone_b = HumanBone::LOWER_LEG_R, .weight = 0.5f, .active = true},
    };

    // Measure initial distances
    std::vector<f32> dist_before;
    for (const auto& c : contacts) {
        Vec3 pa = bone_pos(s.world_a, c.bone_a);
        Vec3 pb = bone_pos(s.world_b, c.bone_b);
        dist_before.push_back(glm::length(pa - pb));
    }

    TwoBodyIKConfig config;
    config.max_iterations = 40;
    two_body_ik_solve(s.skeleton, s.setup_a, s.setup_b,
                      {}, {}, contacts,
                      s.pose_a, s.pose_b, s.world_a, s.world_b, config);

    auto solved_a = compute_world_transforms(s.skeleton, s.pose_a);
    auto solved_b = compute_world_transforms(s.skeleton, s.pose_b);

    for (size_t i = 0; i < contacts.size(); ++i) {
        Vec3 pa = bone_pos(solved_a, contacts[i].bone_a);
        Vec3 pb = bone_pos(solved_b, contacts[i].bone_b);
        f32 dist_after = glm::length(pa - pb);
        REQUIRE(dist_after < dist_before[i]);
    }
}
