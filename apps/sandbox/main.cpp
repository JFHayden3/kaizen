#include "kaizen/platform/window.h"
#include "kaizen/core/skeleton.h"
#include "kaizen/core/ik.h"
#include "kaizen/core/jacobian_ik.h"
#include "kaizen/core/two_body_ik.h"
#include "kaizen/render/debug_draw.h"
#include "kaizen/render/camera.h"
#include "kaizen/render/skeleton_renderer.h"

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

#include <glm/gtc/matrix_transform.hpp>

#include <chrono>
#include <cmath>
#include <limits>

// Apply a procedural idle animation to a pose. phase_offset shifts the timing
// so two characters don't move in lockstep.
static void apply_idle_anim(const kaizen::Skeleton& skel,
                            kaizen::Pose& pose,
                            float t,
                            float phase_offset,
                            const kaizen::Quat& base_root_rot) {
    namespace HB = kaizen::HumanBone;
    float p = t + phase_offset;

    // Subtle weight shift — sway body side to side
    pose.transforms[HB::WORLD_ROOT].translation.x +=
        std::sin(p * 0.8f) * 0.02f;
    pose.transforms[HB::WORLD_ROOT].translation.y +=
        std::sin(p * 1.6f) * 0.005f; // breathing bob

    // Spine sway
    pose.transforms[HB::SPINE_MID].rotation =
        glm::angleAxis(std::sin(p * 0.8f) * 0.03f, kaizen::Vec3{0.0f, 0.0f, 1.0f}) *
        skel.bones[HB::SPINE_MID].rest_rotation;

    pose.transforms[HB::SPINE_UPPER].rotation =
        glm::angleAxis(std::sin(p * 1.0f) * 0.02f, kaizen::Vec3{0.0f, 0.0f, 1.0f}) *
        glm::angleAxis(std::sin(p * 0.6f) * 0.02f, kaizen::Vec3{1.0f, 0.0f, 0.0f}) *
        skel.bones[HB::SPINE_UPPER].rest_rotation;

    // Head look around
    pose.transforms[HB::NECK].rotation =
        glm::angleAxis(std::sin(p * 0.5f) * 0.08f, kaizen::Vec3{0.0f, 1.0f, 0.0f}) *
        glm::angleAxis(std::sin(p * 0.7f) * 0.04f, kaizen::Vec3{1.0f, 0.0f, 0.0f}) *
        skel.bones[HB::NECK].rest_rotation;

    // Arms — slight pendulum swing (opposite phase)
    float arm_swing = std::sin(p * 1.2f) * 0.15f;
    pose.transforms[HB::UPPER_ARM_L].rotation =
        glm::angleAxis(arm_swing, kaizen::Vec3{1.0f, 0.0f, 0.0f}) *
        skel.bones[HB::UPPER_ARM_L].rest_rotation;
    pose.transforms[HB::UPPER_ARM_R].rotation =
        glm::angleAxis(-arm_swing, kaizen::Vec3{1.0f, 0.0f, 0.0f}) *
        skel.bones[HB::UPPER_ARM_R].rest_rotation;

    // Forearms — slight bend
    float forearm_bend = std::sin(p * 1.5f) * 0.1f + 0.1f;
    pose.transforms[HB::FOREARM_L].rotation =
        glm::angleAxis(forearm_bend, kaizen::Vec3{0.0f, 0.0f, 1.0f}) *
        skel.bones[HB::FOREARM_L].rest_rotation;
    pose.transforms[HB::FOREARM_R].rotation =
        glm::angleAxis(-forearm_bend, kaizen::Vec3{0.0f, 0.0f, -1.0f}) *
        skel.bones[HB::FOREARM_R].rest_rotation;

    // Legs — subtle weight shift, alternating knee bend
    float knee_bend_l = (std::sin(p * 0.8f) * 0.5f + 0.5f) * 0.08f;
    float knee_bend_r = (std::sin(p * 0.8f + glm::pi<float>()) * 0.5f + 0.5f) * 0.08f;
    pose.transforms[HB::LOWER_LEG_L].rotation =
        glm::angleAxis(knee_bend_l, kaizen::Vec3{1.0f, 0.0f, 0.0f}) *
        skel.bones[HB::LOWER_LEG_L].rest_rotation;
    pose.transforms[HB::LOWER_LEG_R].rotation =
        glm::angleAxis(knee_bend_r, kaizen::Vec3{1.0f, 0.0f, 0.0f}) *
        skel.bones[HB::LOWER_LEG_R].rest_rotation;
}

// --- Ray casting helpers for click-and-drag bone manipulation ---

static void screen_to_ray(float mouse_x, float mouse_y,
                          int win_w, int win_h,
                          const kaizen::Mat4& view,
                          const kaizen::Mat4& projection,
                          kaizen::Vec3& origin,
                          kaizen::Vec3& dir) {
    float ndc_x = (2.0f * mouse_x) / static_cast<float>(win_w) - 1.0f;
    float ndc_y = 1.0f - (2.0f * mouse_y) / static_cast<float>(win_h);

    kaizen::Mat4 inv_vp = glm::inverse(projection * view);

    kaizen::Vec4 near_ndc(ndc_x, ndc_y, -1.0f, 1.0f);
    kaizen::Vec4 far_ndc(ndc_x, ndc_y, 1.0f, 1.0f);

    kaizen::Vec4 near_world = inv_vp * near_ndc;
    kaizen::Vec4 far_world = inv_vp * far_ndc;

    near_world /= near_world.w;
    far_world /= far_world.w;

    origin = kaizen::Vec3(near_world);
    dir = glm::normalize(kaizen::Vec3(far_world) - kaizen::Vec3(near_world));
}

static float ray_sphere_intersect(const kaizen::Vec3& origin,
                                  const kaizen::Vec3& dir,
                                  const kaizen::Vec3& center,
                                  float radius) {
    kaizen::Vec3 oc = origin - center;
    float b = glm::dot(oc, dir);
    float c = glm::dot(oc, oc) - radius * radius;
    float discriminant = b * b - c;
    if (discriminant < 0.0f) return -1.0f;
    float t = -b - std::sqrt(discriminant);
    if (t < 0.0f) t = -b + std::sqrt(discriminant);
    return t >= 0.0f ? t : -1.0f;
}

static bool ray_plane_intersect(const kaizen::Vec3& origin,
                                const kaizen::Vec3& dir,
                                const kaizen::Vec3& plane_point,
                                const kaizen::Vec3& plane_normal,
                                kaizen::Vec3& hit_point) {
    float denom = glm::dot(plane_normal, dir);
    if (std::abs(denom) < 1e-6f) return false;
    float t = glm::dot(plane_point - origin, plane_normal) / denom;
    if (t < 0.0f) return false;
    hit_point = origin + dir * t;
    return true;
}

static kaizen::Vec2 world_to_screen(const kaizen::Vec3& world_pos,
                                    const kaizen::Mat4& view,
                                    const kaizen::Mat4& projection,
                                    int win_w, int win_h) {
    kaizen::Vec4 clip = projection * view * kaizen::Vec4(world_pos, 1.0f);
    kaizen::Vec3 ndc = kaizen::Vec3(clip) / clip.w;
    float sx = (ndc.x * 0.5f + 0.5f) * static_cast<float>(win_w);
    float sy = (1.0f - (ndc.y * 0.5f + 0.5f)) * static_cast<float>(win_h);
    return {sx, sy};
}

static void draw_dashed_line(kaizen::DebugDraw& dd,
                             const kaizen::Vec3& from, const kaizen::Vec3& to,
                             const kaizen::Vec3& color,
                             float dash_len = 0.03f, float gap_len = 0.03f) {
    kaizen::Vec3 dir = to - from;
    float total_len = glm::length(dir);
    if (total_len < 1e-6f) return;
    kaizen::Vec3 unit = dir / total_len;
    float pos = 0.0f;
    while (pos < total_len) {
        float dash_end = std::min(pos + dash_len, total_len);
        dd.line(from + unit * pos, from + unit * dash_end, color);
        pos = dash_end + gap_len;
    }
}

static ImU32 vec3_to_imcol(const kaizen::Vec3& c, float alpha = 1.0f) {
    return IM_COL32(static_cast<int>(c.x * 255),
                    static_cast<int>(c.y * 255),
                    static_cast<int>(c.z * 255),
                    static_cast<int>(alpha * 255));
}

// Set up a natural grappling stance: arms forward, elbows bent, slight knee bend.
// Breaks the kinematic singularity where the straight arm chain is collinear with
// the inter-body gap direction (making all rotation Jacobian columns zero in that axis).
static void apply_grappling_stance(const kaizen::Skeleton& skel, kaizen::Pose& pose) {
    namespace HB = kaizen::HumanBone;

    // Raise upper arms forward (~50°) and slightly inward
    pose.transforms[HB::UPPER_ARM_L].rotation =
        glm::angleAxis(0.87f, kaizen::Vec3{0.0f, 1.0f, 0.0f}) *
        glm::angleAxis(-0.25f, kaizen::Vec3{0.0f, 0.0f, 1.0f}) *
        skel.bones[HB::UPPER_ARM_L].rest_rotation;
    pose.transforms[HB::UPPER_ARM_R].rotation =
        glm::angleAxis(-0.87f, kaizen::Vec3{0.0f, 1.0f, 0.0f}) *
        glm::angleAxis(0.25f, kaizen::Vec3{0.0f, 0.0f, 1.0f}) *
        skel.bones[HB::UPPER_ARM_R].rest_rotation;

    // Bend elbows ~85° along hinge axis
    pose.transforms[HB::FOREARM_L].rotation =
        glm::angleAxis(1.5f, kaizen::Vec3{0.0f, 1.0f, 0.0f});
    pose.transforms[HB::FOREARM_R].rotation =
        glm::angleAxis(1.5f, kaizen::Vec3{0.0f, -1.0f, 0.0f});

    // Slight forward lean
    pose.transforms[HB::SPINE_UPPER].rotation =
        glm::angleAxis(0.12f, kaizen::Vec3{1.0f, 0.0f, 0.0f}) *
        skel.bones[HB::SPINE_UPPER].rest_rotation;

    // Slight knee bend
    pose.transforms[HB::LOWER_LEG_L].rotation =
        glm::angleAxis(-0.25f, kaizen::Vec3{1.0f, 0.0f, 0.0f});
    pose.transforms[HB::LOWER_LEG_R].rotation =
        glm::angleAxis(-0.25f, kaizen::Vec3{1.0f, 0.0f, 0.0f});
}

static constexpr float BONE_PICK_RADIUS = 0.07f;

struct BoneDragState {
    bool dragging = false;
    int hovered_index = -1;
    int dragged_index = -1;
    kaizen::Vec3 drag_plane_point;
    kaizen::Vec3 drag_plane_normal;
    kaizen::Vec3 drag_offset;
};

struct TwoBodyDragState {
    bool dragging = false;
    int hovered_body = -1;   // 0=A, 1=B
    int hovered_index = -1;
    int dragged_body = -1;
    int dragged_index = -1;
    kaizen::Vec3 drag_plane_point;
    kaizen::Vec3 drag_plane_normal;
    kaizen::Vec3 drag_offset;
};

// Targetable bones for the IK UI
struct TargetableBone {
    kaizen::BoneId bone;
    const char* label;
    kaizen::Vec3 color;
};

static const TargetableBone TARGETABLE_BONES[] = {
    {kaizen::HumanBone::HAND_L,      "Left Hand",   {0.6f,  1.0f,  0.5f}},
    {kaizen::HumanBone::HAND_R,      "Right Hand",  {0.5f,  0.7f,  1.0f}},
    {kaizen::HumanBone::FOREARM_L,   "Left Elbow",  {0.3f,  0.9f,  0.4f}},
    {kaizen::HumanBone::FOREARM_R,   "Right Elbow", {0.3f,  0.55f, 0.9f}},
    {kaizen::HumanBone::FOOT_L,      "Left Foot",   {1.0f,  0.9f,  0.3f}},
    {kaizen::HumanBone::FOOT_R,      "Right Foot",  {1.0f,  0.5f,  0.8f}},
    {kaizen::HumanBone::LOWER_LEG_L, "Left Knee",   {0.9f,  0.7f,  0.15f}},
    {kaizen::HumanBone::LOWER_LEG_R, "Right Knee",  {0.8f,  0.3f,  0.6f}},
    {kaizen::HumanBone::HEAD,        "Head",         {0.6f,  0.95f, 0.95f}},
};
static constexpr size_t NUM_TARGETABLE = sizeof(TARGETABLE_BONES) / sizeof(TARGETABLE_BONES[0]);

// Body colors for two-body mode
static const kaizen::Vec3 BODY_A_COLOR = {0.8f, 0.85f, 1.0f};
static const kaizen::Vec3 BODY_B_COLOR = {1.0f, 0.85f, 0.7f};
static const kaizen::Vec3 CONTACT_COLOR = {1.0f, 0.6f, 0.2f};

int main(int /*argc*/, char* /*argv*/[]) {
    kaizen::Window window({"Kaizen Sandbox", 1280, 720});

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForOpenGL(window.sdl_window(), window.gl_context());
    ImGui_ImplOpenGL3_Init("#version 330");

    // Build skeleton (shared definition)
    auto skeleton = kaizen::build_humanoid_skeleton();

    kaizen::Quat p1_root_rot = kaizen::Quat{1.0f, 0.0f, 0.0f, 0.0f}; // identity
    kaizen::Quat p2_root_rot = glm::angleAxis(glm::pi<float>(), kaizen::Vec3{0.0f, 1.0f, 0.0f});

    kaizen::DebugDraw debug_draw;
    kaizen::OrbitCamera camera;
    bool animate = true;

    // IK state
    auto ik_setup = kaizen::IKSetup::build_humanoid();
    std::vector<kaizen::IKTarget> ik_targets(NUM_TARGETABLE);
    for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
        ik_targets[i].bone = TARGETABLE_BONES[i].bone;
        ik_targets[i].active = true;
    }
    kaizen::IKSolverConfig ik_config;
    ik_config.root_mobility = 1.0f;
    bool ik_mode = false;
    bool targets_initialized = false;
    BoneDragState drag_state;
    std::vector<kaizen::Mat4> last_world;

    // Solver mode: 0 = FABRIK, 1 = Jacobian, 2 = Two-Body
    int solver_mode = 0;
    kaizen::JacobianIKConfig jacobian_config;
    kaizen::Pose persistent_pose = kaizen::Pose::from_rest(skeleton);
    bool jacobian_pose_initialized = false;
    kaizen::u32 last_jacobian_iters = 0;
    float last_solve_ms = 0.0f;

    // Two-body state
    kaizen::TwoBodyIKConfig two_body_config;
    kaizen::IKSetup ik_setup_b = kaizen::IKSetup::build_humanoid();
    kaizen::Pose tb_pose_a = kaizen::Pose::from_rest(skeleton);
    kaizen::Pose tb_pose_b = kaizen::Pose::from_rest(skeleton);
    std::vector<kaizen::Mat4> tb_world_a, tb_world_b;
    bool tb_initialized = false;
    kaizen::TwoBodyIKResult tb_result;
    TwoBodyDragState tb_drag;

    // Two-body IK targets (per body, all targetable bones as anchors)
    std::vector<kaizen::IKTarget> tb_targets_a(NUM_TARGETABLE);
    std::vector<kaizen::IKTarget> tb_targets_b(NUM_TARGETABLE);
    bool tb_targets_initialized = false;

    // Default contacts: basic clinch
    std::vector<kaizen::ContactConstraint> tb_contacts = {
        {kaizen::HumanBone::HAND_L, kaizen::HumanBone::FOREARM_R, 1.0f, true},
        {kaizen::HumanBone::HAND_R, kaizen::HumanBone::UPPER_ARM_L, 1.0f, true},
    };

    SDL_Cursor* cursor_arrow = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_ARROW);
    SDL_Cursor* cursor_hand = SDL_CreateSystemCursor(SDL_SYSTEM_CURSOR_HAND);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_PROGRAM_POINT_SIZE);

    while (!window.should_close()) {
        window.poll_events([&](const SDL_Event& event) {
            ImGui_ImplSDL2_ProcessEvent(&event);

            bool event_consumed = false;

            // Single-body IK drag (FABRIK/Jacobian modes)
            if (ik_mode && solver_mode < 2 && !ImGui::GetIO().WantCaptureMouse && !last_world.empty()) {
                int win_w, win_h;
                SDL_GetWindowSize(window.sdl_window(), &win_w, &win_h);

                if (event.type == SDL_MOUSEMOTION && !drag_state.dragging) {
                    kaizen::Vec3 ray_origin, ray_dir;
                    screen_to_ray(static_cast<float>(event.motion.x),
                                  static_cast<float>(event.motion.y),
                                  win_w, win_h,
                                  camera.view(), camera.projection(),
                                  ray_origin, ray_dir);

                    drag_state.hovered_index = -1;
                    float best_t = std::numeric_limits<float>::max();
                    for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                        kaizen::Vec3 bone_pos = kaizen::Vec3(last_world[TARGETABLE_BONES[i].bone][3]);
                        float t = ray_sphere_intersect(ray_origin, ray_dir, bone_pos, BONE_PICK_RADIUS);
                        if (t >= 0.0f && t < best_t) {
                            best_t = t;
                            drag_state.hovered_index = static_cast<int>(i);
                        }
                    }
                }
                else if (event.type == SDL_MOUSEBUTTONDOWN &&
                         event.button.button == SDL_BUTTON_LEFT &&
                         drag_state.hovered_index >= 0) {
                    int idx = drag_state.hovered_index;
                    drag_state.dragging = true;
                    drag_state.dragged_index = idx;
                    ik_targets[idx].active = true;

                    kaizen::Vec3 bone_pos = kaizen::Vec3(last_world[TARGETABLE_BONES[idx].bone][3]);
                    kaizen::Mat4 inv_view = glm::inverse(camera.view());
                    kaizen::Vec3 cam_forward = -kaizen::Vec3(inv_view[2]);

                    drag_state.drag_plane_point = bone_pos;
                    drag_state.drag_plane_normal = cam_forward;

                    kaizen::Vec3 ray_origin, ray_dir;
                    screen_to_ray(static_cast<float>(event.button.x),
                                  static_cast<float>(event.button.y),
                                  win_w, win_h,
                                  camera.view(), camera.projection(),
                                  ray_origin, ray_dir);

                    kaizen::Vec3 hit;
                    if (ray_plane_intersect(ray_origin, ray_dir,
                                           drag_state.drag_plane_point,
                                           drag_state.drag_plane_normal, hit)) {
                        drag_state.drag_offset = hit - bone_pos;
                    } else {
                        drag_state.drag_offset = kaizen::Vec3(0.0f);
                    }

                    event_consumed = true;
                }
                else if (event.type == SDL_MOUSEMOTION && drag_state.dragging) {
                    kaizen::Vec3 ray_origin, ray_dir;
                    screen_to_ray(static_cast<float>(event.motion.x),
                                  static_cast<float>(event.motion.y),
                                  win_w, win_h,
                                  camera.view(), camera.projection(),
                                  ray_origin, ray_dir);

                    kaizen::Vec3 hit;
                    if (ray_plane_intersect(ray_origin, ray_dir,
                                           drag_state.drag_plane_point,
                                           drag_state.drag_plane_normal, hit)) {
                        ik_targets[drag_state.dragged_index].position = hit - drag_state.drag_offset;
                    }

                    event_consumed = true;
                }
                else if (event.type == SDL_MOUSEBUTTONUP &&
                         event.button.button == SDL_BUTTON_LEFT &&
                         drag_state.dragging) {
                    drag_state.dragging = false;
                    drag_state.dragged_index = -1;
                    event_consumed = true;
                }
            }

            // Two-body IK drag
            if (ik_mode && solver_mode == 2 && !ImGui::GetIO().WantCaptureMouse &&
                !tb_world_a.empty() && !tb_world_b.empty()) {
                int win_w, win_h;
                SDL_GetWindowSize(window.sdl_window(), &win_w, &win_h);

                if (event.type == SDL_MOUSEMOTION && !tb_drag.dragging) {
                    kaizen::Vec3 ray_origin, ray_dir;
                    screen_to_ray(static_cast<float>(event.motion.x),
                                  static_cast<float>(event.motion.y),
                                  win_w, win_h,
                                  camera.view(), camera.projection(),
                                  ray_origin, ray_dir);

                    tb_drag.hovered_index = -1;
                    tb_drag.hovered_body = -1;
                    float best_t = std::numeric_limits<float>::max();

                    // Search both bodies
                    const std::vector<kaizen::Mat4>* worlds[] = {&tb_world_a, &tb_world_b};
                    for (int body = 0; body < 2; ++body) {
                        for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                            kaizen::Vec3 bone_pos = kaizen::Vec3((*worlds[body])[TARGETABLE_BONES[i].bone][3]);
                            float t = ray_sphere_intersect(ray_origin, ray_dir, bone_pos, BONE_PICK_RADIUS);
                            if (t >= 0.0f && t < best_t) {
                                best_t = t;
                                tb_drag.hovered_body = body;
                                tb_drag.hovered_index = static_cast<int>(i);
                            }
                        }
                    }
                }
                else if (event.type == SDL_MOUSEBUTTONDOWN &&
                         event.button.button == SDL_BUTTON_LEFT &&
                         tb_drag.hovered_index >= 0) {
                    tb_drag.dragging = true;
                    tb_drag.dragged_body = tb_drag.hovered_body;
                    tb_drag.dragged_index = tb_drag.hovered_index;

                    const auto& world = (tb_drag.dragged_body == 0) ? tb_world_a : tb_world_b;
                    kaizen::Vec3 bone_pos = kaizen::Vec3(world[TARGETABLE_BONES[tb_drag.dragged_index].bone][3]);

                    kaizen::Mat4 inv_view = glm::inverse(camera.view());
                    kaizen::Vec3 cam_forward = -kaizen::Vec3(inv_view[2]);
                    tb_drag.drag_plane_point = bone_pos;
                    tb_drag.drag_plane_normal = cam_forward;

                    kaizen::Vec3 ray_origin, ray_dir;
                    screen_to_ray(static_cast<float>(event.button.x),
                                  static_cast<float>(event.button.y),
                                  win_w, win_h,
                                  camera.view(), camera.projection(),
                                  ray_origin, ray_dir);

                    kaizen::Vec3 hit;
                    if (ray_plane_intersect(ray_origin, ray_dir,
                                           tb_drag.drag_plane_point,
                                           tb_drag.drag_plane_normal, hit)) {
                        tb_drag.drag_offset = hit - bone_pos;
                    } else {
                        tb_drag.drag_offset = kaizen::Vec3(0.0f);
                    }

                    event_consumed = true;
                }
                else if (event.type == SDL_MOUSEMOTION && tb_drag.dragging) {
                    kaizen::Vec3 ray_origin, ray_dir;
                    screen_to_ray(static_cast<float>(event.motion.x),
                                  static_cast<float>(event.motion.y),
                                  win_w, win_h,
                                  camera.view(), camera.projection(),
                                  ray_origin, ray_dir);

                    kaizen::Vec3 hit;
                    if (ray_plane_intersect(ray_origin, ray_dir,
                                           tb_drag.drag_plane_point,
                                           tb_drag.drag_plane_normal, hit)) {
                        kaizen::Vec3 target_pos = hit - tb_drag.drag_offset;
                        auto& targets = (tb_drag.dragged_body == 0) ? tb_targets_a : tb_targets_b;
                        if (tb_drag.dragged_index >= 0 && tb_drag.dragged_index < static_cast<int>(targets.size())) {
                            targets[tb_drag.dragged_index].position = target_pos;
                        }
                    }

                    event_consumed = true;
                }
                else if (event.type == SDL_MOUSEBUTTONUP &&
                         event.button.button == SDL_BUTTON_LEFT &&
                         tb_drag.dragging) {
                    tb_drag.dragging = false;
                    tb_drag.dragged_body = -1;
                    tb_drag.dragged_index = -1;
                    event_consumed = true;
                }
            }

            if (!event_consumed) {
                camera.process_event(event);
            }
        });

        bool any_hover = false;
        if (ik_mode && solver_mode < 2) {
            any_hover = drag_state.hovered_index >= 0 || drag_state.dragging;
        } else if (ik_mode && solver_mode == 2) {
            any_hover = tb_drag.hovered_index >= 0 || tb_drag.dragging;
        }
        SDL_SetCursor(any_hover ? cursor_hand : cursor_arrow);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Frame timing
        float dt = ImGui::GetIO().DeltaTime;
        float fps = (dt > 0.0f) ? 1.0f / dt : 0.0f;

        // Info panel
        ImGui::Begin("Kaizen Sandbox");
        ImGui::Text("%.1f fps  (%.2f ms)", fps, dt * 1000.0f);
        ImGui::Text("Bones: %d", skeleton.bone_count());
        ImGui::Checkbox("Idle animation", &animate);
        ImGui::Separator();
        ImGui::Text("Camera");
        ImGui::Text("  Yaw: %.1f deg", glm::degrees(camera.yaw()));
        ImGui::Text("  Pitch: %.1f deg", glm::degrees(camera.pitch()));
        ImGui::Text("  Distance: %.1f", camera.distance());
        ImGui::Separator();
        ImGui::Checkbox("IK Mode", &ik_mode);

        if (ik_mode) {
            // Lazy-init target positions from rest pose
            if (!targets_initialized) {
                auto init_pose = kaizen::Pose::from_rest(skeleton);
                auto init_world = kaizen::compute_world_transforms(skeleton, init_pose);
                for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                    ik_targets[i].position =
                        kaizen::Vec3(init_world[TARGETABLE_BONES[i].bone][3]);
                }
                last_world = init_world;
                targets_initialized = true;
            }

            const char* solver_names[] = {"FABRIK", "Jacobian", "Two-Body"};
            ImGui::Combo("Solver", &solver_mode, solver_names, 3);

            if (solver_mode == 0) {
                // FABRIK controls
                int max_iter = static_cast<int>(ik_config.max_iterations);
                ImGui::SliderInt("Max iterations", &max_iter, 1, 30);
                ik_config.max_iterations = static_cast<kaizen::u32>(max_iter);
                ImGui::SliderFloat("Tolerance", &ik_config.tolerance, 0.0001f, 0.01f, "%.4f");
                ImGui::SliderFloat("Root mobility", &ik_config.root_mobility, 0.0f, 1.0f);
            } else if (solver_mode == 1) {
                // Jacobian controls
                int max_iter = static_cast<int>(jacobian_config.max_iterations);
                ImGui::SliderInt("Max iterations", &max_iter, 1, 60);
                jacobian_config.max_iterations = static_cast<kaizen::u32>(max_iter);
                ImGui::SliderFloat("Tolerance", &jacobian_config.tolerance, 0.0001f, 0.01f, "%.4f");
                ImGui::SliderFloat("Damping", &jacobian_config.damping, 0.01f, 2.0f);
                ImGui::SliderFloat("Max step", &jacobian_config.max_step, 0.01f, 0.5f);
                ImGui::Checkbox("Root translates", &jacobian_config.root_translates);

                ImGui::Text("Iterations: %u  Solve: %.2f ms", last_jacobian_iters, last_solve_ms);

                if (ImGui::Button("Reset Pose")) {
                    persistent_pose = kaizen::Pose::from_rest(skeleton);
                    jacobian_pose_initialized = false;
                }
            } else {
                // Two-Body controls
                int max_iter = static_cast<int>(two_body_config.max_iterations);
                ImGui::SliderInt("Max iterations", &max_iter, 1, 60);
                two_body_config.max_iterations = static_cast<kaizen::u32>(max_iter);
                ImGui::SliderFloat("Tolerance", &two_body_config.tolerance, 0.0001f, 0.01f, "%.4f");
                ImGui::SliderFloat("Damping", &two_body_config.damping, 0.01f, 2.0f);
                ImGui::SliderFloat("Max step", &two_body_config.max_step, 0.01f, 0.5f);

                ImGui::Separator();
                ImGui::Text("Root Translation");
                ImGui::Checkbox("Root translates A", &two_body_config.root_translates_a);
                ImGui::SameLine();
                ImGui::Checkbox("Root translates B", &two_body_config.root_translates_b);
                ImGui::SliderFloat("Root weight A", &two_body_config.root_weight_a, 0.0f, 1.0f);
                ImGui::SliderFloat("Root weight B", &two_body_config.root_weight_b, 0.0f, 1.0f);

                ImGui::Separator();
                ImGui::Text("Contacts");
                ImGui::PushItemWidth(200.0f);
                for (size_t i = 0; i < tb_contacts.size(); ++i) {
                    ImGui::PushID(static_cast<int>(i));
                    auto& c = tb_contacts[i];

                    // Find bone names for display
                    const char* name_a = "?";
                    const char* name_b = "?";
                    for (size_t j = 0; j < NUM_TARGETABLE; ++j) {
                        if (TARGETABLE_BONES[j].bone == c.bone_a) name_a = TARGETABLE_BONES[j].label;
                        if (TARGETABLE_BONES[j].bone == c.bone_b) name_b = TARGETABLE_BONES[j].label;
                    }

                    char label[128];
                    snprintf(label, sizeof(label), "A:%s -> B:%s", name_a, name_b);
                    if (ImGui::TreeNode(label)) {
                        ImGui::Checkbox("Active", &c.active);
                        ImGui::SliderFloat("Weight", &c.weight, 0.0f, 2.0f);
                        ImGui::TreePop();
                    }
                    ImGui::PopID();
                }
                ImGui::PopItemWidth();

                ImGui::Separator();
                ImGui::Text("Iterations: %u  Max error: %.4f  Solve: %.2f ms",
                            tb_result.iterations, tb_result.max_error, last_solve_ms);

                if (ImGui::Button("Reset Poses")) {
                    tb_initialized = false;
                }
            }

            // Per-bone target controls (FABRIK/Jacobian only)
            if (solver_mode < 2) {
                ImGui::PushItemWidth(250.0f);
                for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                    ImGui::PushID(static_cast<int>(i));
                    if (ImGui::TreeNode(TARGETABLE_BONES[i].label)) {
                        ImGui::Checkbox("Active", &ik_targets[i].active);
                        ImGui::SliderFloat3("Position", &ik_targets[i].position.x, -2.0f, 2.0f);
                        ImGui::SliderFloat("Weight", &ik_targets[i].weight, 0.0f, 1.0f);
                        ImGui::TreePop();
                    }
                    ImGui::PopID();
                }
                ImGui::PopItemWidth();
            }
        }

        ImGui::Separator();
        if (ik_mode) {
            ImGui::TextWrapped("Click and drag bone handles to set IK targets. "
                               "Drag empty space to orbit. Scroll to zoom.");
        } else {
            ImGui::TextWrapped("Left-drag to orbit, scroll to zoom");
        }
        ImGui::End();

        // Update camera
        int w, h;
        SDL_GL_GetDrawableSize(window.sdl_window(), &w, &h);
        float aspect = (h > 0) ? static_cast<float>(w) / static_cast<float>(h) : 1.0f;
        camera.update(aspect);

        // Submit geometry
        kaizen::SkeletonRenderer::draw_ground_grid(debug_draw);

        if (ik_mode && solver_mode < 2) {
            // Single-body IK modes (FABRIK / Jacobian)
            std::vector<kaizen::Mat4> world;

            if (solver_mode == 0) {
                // FABRIK: rebuild from rest each frame
                auto pose = kaizen::Pose::from_rest(skeleton);
                world = kaizen::compute_world_transforms(skeleton, pose);
                kaizen::ik_solve(skeleton, ik_setup, ik_targets, pose, world, ik_config);
                world = kaizen::compute_world_transforms(skeleton, pose);
            } else {
                // Jacobian: use persistent pose across frames
                if (!jacobian_pose_initialized) {
                    persistent_pose = kaizen::Pose::from_rest(skeleton);
                    jacobian_pose_initialized = true;
                }
                world = kaizen::compute_world_transforms(skeleton, persistent_pose);
                auto solve_start = std::chrono::steady_clock::now();
                last_jacobian_iters = kaizen::jacobian_ik_solve(
                    skeleton, ik_setup, ik_targets, persistent_pose, world, jacobian_config);
                last_solve_ms = std::chrono::duration<float, std::milli>(
                    std::chrono::steady_clock::now() - solve_start).count();
                world = kaizen::compute_world_transforms(skeleton, persistent_pose);
            }

            last_world = world;

            // While dragging, non-dragged targets follow solved positions so they
            // don't fight the drag. Once released, all targets freeze in place.
            if (drag_state.dragging) {
                for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                    if (drag_state.dragged_index == static_cast<int>(i))
                        continue;
                    ik_targets[i].position = kaizen::Vec3(world[TARGETABLE_BONES[i].bone][3]);
                }
            }

            kaizen::SkeletonRenderer::draw(skeleton, world, debug_draw);

            // Draw dashed error lines from bone to IK target
            for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                if (!ik_targets[i].active) continue;
                kaizen::Vec3 color = TARGETABLE_BONES[i].color;
                kaizen::Vec3 bone_pos = kaizen::Vec3(world[TARGETABLE_BONES[i].bone][3]);
                draw_dashed_line(debug_draw, bone_pos, ik_targets[i].position, color);
            }
        } else if (ik_mode && solver_mode == 2) {
            // Two-Body IK mode
            if (!tb_initialized) {
                tb_pose_a = kaizen::Pose::from_rest(skeleton);
                tb_pose_a.transforms[kaizen::HumanBone::WORLD_ROOT].translation = kaizen::Vec3{-0.8f, 1.10f, 0.0f};
                tb_pose_a.transforms[kaizen::HumanBone::WORLD_ROOT].rotation = p1_root_rot;
                apply_grappling_stance(skeleton, tb_pose_a);

                tb_pose_b = kaizen::Pose::from_rest(skeleton);
                tb_pose_b.transforms[kaizen::HumanBone::WORLD_ROOT].translation = kaizen::Vec3{0.8f, 1.10f, 0.0f};
                tb_pose_b.transforms[kaizen::HumanBone::WORLD_ROOT].rotation = p2_root_rot;
                apply_grappling_stance(skeleton, tb_pose_b);

                tb_world_a = kaizen::compute_world_transforms(skeleton, tb_pose_a);
                tb_world_b = kaizen::compute_world_transforms(skeleton, tb_pose_b);
                tb_initialized = true;
                tb_targets_initialized = false;
            }

            // Initialize anchor targets from current world positions
            if (!tb_targets_initialized && !tb_world_a.empty()) {
                for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                    tb_targets_a[i].bone = TARGETABLE_BONES[i].bone;
                    tb_targets_a[i].active = true;
                    tb_targets_a[i].weight = 1.0f;
                    tb_targets_a[i].position = kaizen::Vec3(tb_world_a[TARGETABLE_BONES[i].bone][3]);

                    tb_targets_b[i].bone = TARGETABLE_BONES[i].bone;
                    tb_targets_b[i].active = true;
                    tb_targets_b[i].weight = 1.0f;
                    tb_targets_b[i].position = kaizen::Vec3(tb_world_b[TARGETABLE_BONES[i].bone][3]);
                }
                tb_targets_initialized = true;
            }

            tb_world_a = kaizen::compute_world_transforms(skeleton, tb_pose_a);
            tb_world_b = kaizen::compute_world_transforms(skeleton, tb_pose_b);

            {
                auto solve_start = std::chrono::steady_clock::now();
                tb_result = kaizen::two_body_ik_solve(
                    skeleton, ik_setup, ik_setup_b,
                    tb_targets_a, tb_targets_b, tb_contacts,
                    tb_pose_a, tb_pose_b, tb_world_a, tb_world_b,
                    two_body_config);
                last_solve_ms = std::chrono::duration<float, std::milli>(
                    std::chrono::steady_clock::now() - solve_start).count();
            }

            tb_world_a = kaizen::compute_world_transforms(skeleton, tb_pose_a);
            tb_world_b = kaizen::compute_world_transforms(skeleton, tb_pose_b);

            // Update non-dragged anchor targets to current solved positions
            // (prevents anchors from fighting the solver on subsequent frames)
            for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                bool is_dragged_a = tb_drag.dragging && tb_drag.dragged_body == 0 &&
                                    tb_drag.dragged_index == static_cast<int>(i);
                bool is_dragged_b = tb_drag.dragging && tb_drag.dragged_body == 1 &&
                                    tb_drag.dragged_index == static_cast<int>(i);
                if (!is_dragged_a)
                    tb_targets_a[i].position = kaizen::Vec3(tb_world_a[TARGETABLE_BONES[i].bone][3]);
                if (!is_dragged_b)
                    tb_targets_b[i].position = kaizen::Vec3(tb_world_b[TARGETABLE_BONES[i].bone][3]);
            }

            kaizen::SkeletonRenderer::draw(skeleton, tb_world_a, debug_draw, BODY_A_COLOR);
            kaizen::SkeletonRenderer::draw(skeleton, tb_world_b, debug_draw, BODY_B_COLOR);

            // Draw contact constraint lines
            for (const auto& c : tb_contacts) {
                if (!c.active) continue;
                kaizen::Vec3 pa = kaizen::Vec3(tb_world_a[c.bone_a][3]);
                kaizen::Vec3 pb = kaizen::Vec3(tb_world_b[c.bone_b][3]);
                draw_dashed_line(debug_draw, pa, pb, CONTACT_COLOR);
            }

            // Draw drag target line if dragging
            if (tb_drag.dragging && tb_drag.dragged_index >= 0) {
                const auto& world = (tb_drag.dragged_body == 0) ? tb_world_a : tb_world_b;
                const auto& targets = (tb_drag.dragged_body == 0) ? tb_targets_a : tb_targets_b;
                kaizen::Vec3 bone_pos = kaizen::Vec3(world[TARGETABLE_BONES[tb_drag.dragged_index].bone][3]);
                kaizen::Vec3 color = TARGETABLE_BONES[tb_drag.dragged_index].color;
                draw_dashed_line(debug_draw, bone_pos, targets[tb_drag.dragged_index].position, color);
            }
        } else {
            // Two characters with optional idle animation
            auto pose1 = kaizen::Pose::from_rest(skeleton);
            pose1.transforms[kaizen::HumanBone::WORLD_ROOT].translation = kaizen::Vec3{-0.8f, 1.10f, 0.0f};
            pose1.transforms[kaizen::HumanBone::WORLD_ROOT].rotation = p1_root_rot;

            auto pose2 = kaizen::Pose::from_rest(skeleton);
            pose2.transforms[kaizen::HumanBone::WORLD_ROOT].translation = kaizen::Vec3{0.8f, 1.10f, 0.0f};
            pose2.transforms[kaizen::HumanBone::WORLD_ROOT].rotation = p2_root_rot;

            if (animate) {
                float t = static_cast<float>(SDL_GetTicks()) / 1000.0f;
                apply_idle_anim(skeleton, pose1, t, 0.0f, p1_root_rot);
                apply_idle_anim(skeleton, pose2, t, 1.7f, p2_root_rot);
            }

            auto world1 = kaizen::compute_world_transforms(skeleton, pose1);
            auto world2 = kaizen::compute_world_transforms(skeleton, pose2);

            kaizen::SkeletonRenderer::draw(skeleton, world1, debug_draw, BODY_A_COLOR);
            kaizen::SkeletonRenderer::draw(skeleton, world2, debug_draw, BODY_B_COLOR);
        }

        // Draw grab handles and hover tooltip via ImGui (screen-space, no depth issues)
        if (ik_mode && solver_mode < 2 && !last_world.empty()) {
            int win_w, win_h;
            SDL_GetWindowSize(window.sdl_window(), &win_w, &win_h);
            ImDrawList* fg = ImGui::GetForegroundDrawList();

            for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                kaizen::Vec3 bone_pos = kaizen::Vec3(last_world[TARGETABLE_BONES[i].bone][3]);
                kaizen::Vec2 sp = world_to_screen(bone_pos, camera.view(), camera.projection(), win_w, win_h);
                ImVec2 center(sp.x, sp.y);

                kaizen::Vec3 color = TARGETABLE_BONES[i].color;
                float radius = 7.0f;
                float outline = 2.0f;

                if (drag_state.dragging && drag_state.dragged_index == static_cast<int>(i)) {
                    color = kaizen::Vec3(1.0f);
                    radius = 9.0f;
                } else if (drag_state.hovered_index == static_cast<int>(i)) {
                    color = glm::mix(color, kaizen::Vec3(1.0f), 0.4f);
                    radius = 8.0f;
                }

                fg->AddCircleFilled(center, radius + outline, IM_COL32(15, 15, 15, 220));
                fg->AddCircleFilled(center, radius, vec3_to_imcol(color));
            }

            // Hover tooltip
            int tip_idx = drag_state.dragging ? drag_state.dragged_index : drag_state.hovered_index;
            if (tip_idx >= 0) {
                kaizen::Vec3 bone_pos = kaizen::Vec3(last_world[TARGETABLE_BONES[tip_idx].bone][3]);
                kaizen::Vec2 sp = world_to_screen(bone_pos, camera.view(), camera.projection(), win_w, win_h);

                const char* label = TARGETABLE_BONES[tip_idx].label;
                ImVec2 text_size = ImGui::CalcTextSize(label);
                ImVec2 pos(sp.x - text_size.x * 0.5f, sp.y - text_size.y - 16.0f);
                ImVec2 pad(4.0f, 2.0f);
                fg->AddRectFilled(ImVec2(pos.x - pad.x, pos.y - pad.y),
                                  ImVec2(pos.x + text_size.x + pad.x, pos.y + text_size.y + pad.y),
                                  IM_COL32(0, 0, 0, 180), 3.0f);
                fg->AddText(pos, IM_COL32(255, 255, 255, 255), label);
            }
        }

        // Two-body grab handles
        if (ik_mode && solver_mode == 2 && !tb_world_a.empty()) {
            int win_w, win_h;
            SDL_GetWindowSize(window.sdl_window(), &win_w, &win_h);
            ImDrawList* fg = ImGui::GetForegroundDrawList();

            const std::vector<kaizen::Mat4>* worlds[] = {&tb_world_a, &tb_world_b};
            const kaizen::Vec3 body_tints[] = {BODY_A_COLOR, BODY_B_COLOR};

            for (int body = 0; body < 2; ++body) {
                for (size_t i = 0; i < NUM_TARGETABLE; ++i) {
                    kaizen::Vec3 bone_pos = kaizen::Vec3((*worlds[body])[TARGETABLE_BONES[i].bone][3]);
                    kaizen::Vec2 sp = world_to_screen(bone_pos, camera.view(), camera.projection(), win_w, win_h);
                    ImVec2 center(sp.x, sp.y);

                    kaizen::Vec3 color = TARGETABLE_BONES[i].color * body_tints[body];
                    float radius = 6.0f;
                    float outline = 1.5f;

                    if (tb_drag.dragging && tb_drag.dragged_body == body &&
                        tb_drag.dragged_index == static_cast<int>(i)) {
                        color = kaizen::Vec3(1.0f);
                        radius = 9.0f;
                    } else if (tb_drag.hovered_body == body &&
                               tb_drag.hovered_index == static_cast<int>(i)) {
                        color = glm::mix(color, kaizen::Vec3(1.0f), 0.4f);
                        radius = 7.0f;
                    }

                    fg->AddCircleFilled(center, radius + outline, IM_COL32(15, 15, 15, 180));
                    fg->AddCircleFilled(center, radius, vec3_to_imcol(color));
                }
            }

            // Hover tooltip
            int tip_body = tb_drag.dragging ? tb_drag.dragged_body : tb_drag.hovered_body;
            int tip_idx = tb_drag.dragging ? tb_drag.dragged_index : tb_drag.hovered_index;
            if (tip_idx >= 0 && tip_body >= 0) {
                kaizen::Vec3 bone_pos = kaizen::Vec3((*worlds[tip_body])[TARGETABLE_BONES[tip_idx].bone][3]);
                kaizen::Vec2 sp = world_to_screen(bone_pos, camera.view(), camera.projection(), win_w, win_h);

                char label[64];
                snprintf(label, sizeof(label), "%c: %s",
                         tip_body == 0 ? 'A' : 'B',
                         TARGETABLE_BONES[tip_idx].label);
                ImVec2 text_size = ImGui::CalcTextSize(label);
                ImVec2 pos(sp.x - text_size.x * 0.5f, sp.y - text_size.y - 16.0f);
                ImVec2 pad(4.0f, 2.0f);
                fg->AddRectFilled(ImVec2(pos.x - pad.x, pos.y - pad.y),
                                  ImVec2(pos.x + text_size.x + pad.x, pos.y + text_size.y + pad.y),
                                  IM_COL32(0, 0, 0, 180), 3.0f);
                fg->AddText(pos, IM_COL32(255, 255, 255, 255), label);
            }
        }

        // Render
        ImGui::Render();
        glViewport(0, 0, w, h);
        glClearColor(0.12f, 0.12f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        debug_draw.render(camera.view_projection());

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        window.swap();
    }

    SDL_FreeCursor(cursor_arrow);
    SDL_FreeCursor(cursor_hand);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
