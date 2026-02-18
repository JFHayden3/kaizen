#pragma once

#include "kaizen/core/math.h"

#include <SDL.h>

namespace kaizen {

class OrbitCamera {
public:
    OrbitCamera();

    void process_event(const SDL_Event& event);
    void update(float aspect_ratio);

    Mat4 view() const { return view_; }
    Mat4 projection() const { return projection_; }
    Mat4 view_projection() const { return projection_ * view_; }

    float yaw() const { return yaw_; }
    float pitch() const { return pitch_; }
    float distance() const { return distance_; }
    Vec3 focus() const { return focus_; }

private:
    Vec3 focus_{0.0f, 0.95f, 0.0f};  // center of mat at hip height
    float yaw_ = 0.0f;               // radians
    float pitch_ = 0.3f;             // radians, slight downward angle
    float distance_ = 5.0f;

    float fov_ = glm::radians(45.0f);
    float near_ = 0.1f;
    float far_ = 100.0f;

    Mat4 view_{1.0f};
    Mat4 projection_{1.0f};

    bool dragging_ = false;
};

} // namespace kaizen
