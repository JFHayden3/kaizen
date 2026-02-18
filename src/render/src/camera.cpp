#include "kaizen/render/camera.h"

#include <imgui.h>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

namespace kaizen {

OrbitCamera::OrbitCamera() {
}

void OrbitCamera::process_event(const SDL_Event& event) {
    if (ImGui::GetIO().WantCaptureMouse) return;

    switch (event.type) {
        case SDL_MOUSEBUTTONDOWN:
            if (event.button.button == SDL_BUTTON_LEFT) {
                dragging_ = true;
            }
            break;

        case SDL_MOUSEBUTTONUP:
            if (event.button.button == SDL_BUTTON_LEFT) {
                dragging_ = false;
            }
            break;

        case SDL_MOUSEMOTION:
            if (dragging_) {
                constexpr float sensitivity = 0.005f;
                yaw_ -= event.motion.xrel * sensitivity;
                pitch_ -= event.motion.yrel * sensitivity;
                pitch_ = std::clamp(pitch_, -glm::half_pi<float>() + 0.01f,
                                             glm::half_pi<float>() - 0.01f);
            }
            break;

        case SDL_MOUSEWHEEL:
            distance_ -= event.wheel.y * 0.3f;
            distance_ = std::clamp(distance_, 1.0f, 20.0f);
            break;
    }
}

void OrbitCamera::update(float aspect_ratio) {
    // Spherical to cartesian
    float x = distance_ * std::cos(pitch_) * std::sin(yaw_);
    float y = distance_ * std::sin(pitch_);
    float z = distance_ * std::cos(pitch_) * std::cos(yaw_);

    Vec3 eye = focus_ + Vec3{x, y, z};

    view_ = glm::lookAt(eye, focus_, Vec3{0.0f, 1.0f, 0.0f});
    projection_ = glm::perspective(fov_, aspect_ratio, near_, far_);
}

} // namespace kaizen
