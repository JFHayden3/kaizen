#include "kaizen/render/skeleton_renderer.h"

namespace kaizen {

static bool is_head_contact(BoneId id) {
    return id >= HumanBone::HEAD_CONTACT_FIRST && id <= HumanBone::HEAD_CONTACT_LAST;
}

static Vec3 bone_pos(const std::vector<Mat4>& w, BoneId id) {
    return Vec3(w[id][3]);
}

// Per-bone distinct color
static Vec3 bone_color(BoneId id) {
    namespace HB = HumanBone;
    switch (id) {
        // Spine
        case HB::WORLD_ROOT:   return {0.85f, 0.85f, 0.85f};
        case HB::SPINE_MID:    return {0.85f, 0.85f, 0.85f};
        case HB::SPINE_UPPER:  return {1.0f,  1.0f,  1.0f};
        case HB::PELVIS:       return {0.65f, 0.65f, 0.65f};
        // Head/neck
        case HB::NECK:         return {0.5f,  0.8f,  0.8f};
        case HB::HEAD:         return {0.6f,  0.95f, 0.95f};
        // Left arm: green gradient (dark → bright)
        case HB::CLAVICLE_L:   return {0.15f, 0.5f,  0.2f};
        case HB::UPPER_ARM_L:  return {0.2f,  0.7f,  0.3f};
        case HB::FOREARM_L:    return {0.3f,  0.9f,  0.4f};
        case HB::HAND_L:       return {0.6f,  1.0f,  0.5f};
        // Right arm: blue gradient (dark → bright)
        case HB::CLAVICLE_R:   return {0.15f, 0.25f, 0.55f};
        case HB::UPPER_ARM_R:  return {0.2f,  0.4f,  0.75f};
        case HB::FOREARM_R:    return {0.3f,  0.55f, 0.9f};
        case HB::HAND_R:       return {0.5f,  0.7f,  1.0f};
        // Left leg: yellow-orange gradient (dark → bright)
        case HB::UPPER_LEG_L:  return {0.7f,  0.5f,  0.1f};
        case HB::LOWER_LEG_L:  return {0.9f,  0.7f,  0.15f};
        case HB::FOOT_L:       return {1.0f,  0.9f,  0.3f};
        // Right leg: magenta-pink gradient (dark → bright)
        case HB::UPPER_LEG_R:  return {0.6f,  0.15f, 0.4f};
        case HB::LOWER_LEG_R:  return {0.8f,  0.3f,  0.6f};
        case HB::FOOT_R:       return {1.0f,  0.5f,  0.8f};
        // Hands (fingertip end of limb)
        case HB::FINGERS_L:    return {0.7f,  1.0f,  0.6f};
        case HB::FINGERS_R:    return {0.6f,  0.8f,  1.0f};
        // Feet (toe end of limb)
        case HB::TOES_L:       return {1.0f,  0.95f, 0.4f};
        case HB::TOES_R:       return {1.0f,  0.6f,  0.85f};
        default:               return {0.7f,  0.7f,  0.7f};
    }
}

static void draw_head_wireframe(const std::vector<Mat4>& world_transforms,
                                 DebugDraw& debug, const Vec3& color) {
    auto p = [&](BoneId id) { return bone_pos(world_transforms, id); };

    // Sagittal outline (front-to-back over top)
    debug.line(p(HumanBone::FOREHEAD),    p(HumanBone::HEAD_TOP),  color, 3.0f);
    debug.line(p(HumanBone::HEAD_TOP),    p(HumanBone::HEAD_BACK), color, 3.0f);

    // Front face outline
    debug.line(p(HumanBone::FOREHEAD),    p(HumanBone::JAW_L),     color, 3.0f);
    debug.line(p(HumanBone::FOREHEAD),    p(HumanBone::JAW_R),     color, 3.0f);
    debug.line(p(HumanBone::JAW_L),       p(HumanBone::CHIN),      color, 3.0f);
    debug.line(p(HumanBone::JAW_R),       p(HumanBone::CHIN),      color, 3.0f);

    // Sides connecting front to back
    debug.line(p(HumanBone::JAW_L),       p(HumanBone::HEAD_SIDE_L), color, 3.0f);
    debug.line(p(HumanBone::JAW_R),       p(HumanBone::HEAD_SIDE_R), color, 3.0f);
    debug.line(p(HumanBone::HEAD_SIDE_L), p(HumanBone::HEAD_BACK),   color, 3.0f);
    debug.line(p(HumanBone::HEAD_SIDE_R), p(HumanBone::HEAD_BACK),   color, 3.0f);

    // Top ring
    debug.line(p(HumanBone::HEAD_TOP),    p(HumanBone::HEAD_SIDE_L), color, 3.0f);
    debug.line(p(HumanBone::HEAD_TOP),    p(HumanBone::HEAD_SIDE_R), color, 3.0f);

    // Neck to chin (connects head wireframe to body)
    debug.line(p(HumanBone::HEAD),        p(HumanBone::CHIN),        color, 3.0f);
}

void SkeletonRenderer::draw(const Skeleton& skeleton,
                             const std::vector<Mat4>& world_transforms,
                             DebugDraw& debug,
                             const Vec3& tint) {
    for (BoneId i = 0; i < skeleton.bone_count(); ++i) {
        if (is_head_contact(i)) continue; // drawn as wireframe below

        Vec3 color = bone_color(i) * tint;
        Vec3 pos = Vec3(world_transforms[i][3]);
        debug.point(pos, color, 10.0f);

        if (skeleton.bones[i].parent != INVALID_BONE) {
            Vec3 parent_pos = Vec3(world_transforms[skeleton.bones[i].parent][3]);
            debug.line(parent_pos, pos, color, 3.0f);
        }
    }

    Vec3 head_color = bone_color(HumanBone::HEAD) * tint;
    draw_head_wireframe(world_transforms, debug, head_color);
}

void SkeletonRenderer::draw_ground_grid(DebugDraw& debug, float size, float step) {
    Vec3 grid_color{0.3f, 0.3f, 0.3f};
    Vec3 x_axis_color{0.6f, 0.2f, 0.2f};
    Vec3 z_axis_color{0.2f, 0.2f, 0.6f};

    for (float i = -size; i <= size; i += step) {
        // Lines parallel to Z axis
        Vec3 color = (i == 0.0f) ? x_axis_color : grid_color;
        debug.line({i, 0.0f, -size}, {i, 0.0f, size}, color);

        // Lines parallel to X axis
        color = (i == 0.0f) ? z_axis_color : grid_color;
        debug.line({-size, 0.0f, i}, {size, 0.0f, i}, color);
    }
}

} // namespace kaizen
