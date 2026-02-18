#pragma once

#include "kaizen/core/ik.h"

#include <algorithm>
#include <cmath>

namespace kaizen::detail {

inline Vec3 quat_rotate(const Quat& q, const Vec3& v) {
    Vec3 u(q.x, q.y, q.z);
    f32 s = q.w;
    return 2.0f * glm::dot(u, v) * u + (s * s - glm::dot(u, u)) * v +
           2.0f * s * glm::cross(u, v);
}

inline Quat rotation_between(const Vec3& from, const Vec3& to) {
    Vec3 f = glm::normalize(from);
    Vec3 t = glm::normalize(to);
    f32 d = glm::dot(f, t);

    if (d >= 1.0f - 1e-6f) {
        return Quat{1.0f, 0.0f, 0.0f, 0.0f};
    }
    if (d <= -1.0f + 1e-6f) {
        Vec3 axis = glm::cross(Vec3{1.0f, 0.0f, 0.0f}, f);
        if (glm::dot(axis, axis) < 1e-6f) {
            axis = glm::cross(Vec3{0.0f, 1.0f, 0.0f}, f);
        }
        return glm::angleAxis(glm::pi<f32>(), glm::normalize(axis));
    }

    Vec3 axis = glm::cross(f, t);
    f32 w = 1.0f + d;
    return glm::normalize(Quat{w, axis.x, axis.y, axis.z});
}

inline Quat apply_hinge_constraint(const Quat& q, const HingeLimit& limit) {
    Vec3 v(q.x, q.y, q.z);
    f32 p = glm::dot(v, limit.axis);
    f32 angle = 2.0f * std::atan2(p, q.w);

    if (angle > glm::pi<f32>()) angle -= 2.0f * glm::pi<f32>();
    if (angle < -glm::pi<f32>()) angle += 2.0f * glm::pi<f32>();

    angle = std::clamp(angle, limit.min_angle, limit.max_angle);
    return glm::angleAxis(angle, limit.axis);
}

inline Quat apply_cone_constraint(const Quat& q, const ConeLimit& limit) {
    Vec3 rotated = quat_rotate(q, limit.axis);
    f32 d = std::clamp(glm::dot(rotated, limit.axis), -1.0f, 1.0f);
    f32 angle = std::acos(d);

    if (angle <= limit.half_angle) return q;

    Vec3 cross = glm::cross(limit.axis, rotated);
    if (glm::dot(cross, cross) < 1e-12f) {
        cross = glm::cross(limit.axis, Vec3{1.0f, 0.0f, 0.0f});
        if (glm::dot(cross, cross) < 1e-6f) {
            cross = glm::cross(limit.axis, Vec3{0.0f, 1.0f, 0.0f});
        }
    }
    Vec3 rot_axis = glm::normalize(cross);
    Vec3 clamped_dir =
        quat_rotate(glm::angleAxis(limit.half_angle, rot_axis), limit.axis);
    Quat correction = rotation_between(rotated, clamped_dir);
    return correction * q;
}

inline Quat apply_constraint(const JointConstraint& constraint, const Quat& q) {
    if (auto* hinge = std::get_if<HingeLimit>(&constraint)) {
        return apply_hinge_constraint(q, *hinge);
    }
    if (auto* cone = std::get_if<ConeLimit>(&constraint)) {
        return apply_cone_constraint(q, *cone);
    }
    return q;
}

} // namespace kaizen::detail
