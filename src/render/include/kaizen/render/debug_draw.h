#pragma once

#include "kaizen/core/math.h"

#include <vector>

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

namespace kaizen {

class DebugDraw {
public:
    DebugDraw();
    ~DebugDraw();

    DebugDraw(const DebugDraw&) = delete;
    DebugDraw& operator=(const DebugDraw&) = delete;

    void line(const Vec3& from, const Vec3& to, const Vec3& color, float width = 1.0f);
    void point(const Vec3& pos, const Vec3& color, float size = 6.0f);

    // Uploads geometry, draws, and clears buffers
    void render(const Mat4& view_projection);

private:
    struct Vertex {
        Vec3 position;
        Vec3 color;
    };

    struct PointEntry {
        Vec3 position;
        Vec3 color;
        float size;
    };

    std::vector<Vertex> line_vertices_;
    std::vector<Vertex> thick_line_vertices_;
    float thick_line_width_ = 3.0f;
    std::vector<PointEntry> points_;

    GLuint shader_ = 0;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLint mvp_loc_ = -1;

    void init_gl();
};

} // namespace kaizen
