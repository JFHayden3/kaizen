#include "kaizen/render/debug_draw.h"

#include <glm/gtc/type_ptr.hpp>
#include <cstdio>

namespace kaizen {

static constexpr const char* VERT_SRC = R"glsl(
#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_color;

uniform mat4 u_mvp;

out vec3 v_color;

void main() {
    gl_Position = u_mvp * vec4(a_position, 1.0);
    v_color = a_color;
}
)glsl";

static constexpr const char* FRAG_SRC = R"glsl(
#version 330 core
in vec3 v_color;
out vec4 frag_color;

void main() {
    frag_color = vec4(v_color, 1.0);
}
)glsl";

static GLuint compile_shader(GLenum type, const char* src) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        std::fprintf(stderr, "Shader compile error: %s\n", log);
    }
    return shader;
}

DebugDraw::DebugDraw() {
    init_gl();
}

DebugDraw::~DebugDraw() {
    if (vbo_) glDeleteBuffers(1, &vbo_);
    if (vao_) glDeleteVertexArrays(1, &vao_);
    if (shader_) glDeleteProgram(shader_);
}

void DebugDraw::init_gl() {
    GLuint vert = compile_shader(GL_VERTEX_SHADER, VERT_SRC);
    GLuint frag = compile_shader(GL_FRAGMENT_SHADER, FRAG_SRC);

    shader_ = glCreateProgram();
    glAttachShader(shader_, vert);
    glAttachShader(shader_, frag);
    glLinkProgram(shader_);

    GLint ok = 0;
    glGetProgramiv(shader_, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(shader_, sizeof(log), nullptr, log);
        std::fprintf(stderr, "Shader link error: %s\n", log);
    }

    glDeleteShader(vert);
    glDeleteShader(frag);

    mvp_loc_ = glGetUniformLocation(shader_, "u_mvp");

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);

    // position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, position)));
    // color
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, color)));

    glBindVertexArray(0);
}

void DebugDraw::line(const Vec3& from, const Vec3& to, const Vec3& color, float width) {
    auto& buf = (width > 1.5f) ? thick_line_vertices_ : line_vertices_;
    buf.push_back({from, color});
    buf.push_back({to, color});
}

void DebugDraw::point(const Vec3& pos, const Vec3& color, float size) {
    points_.push_back({pos, color, size});
}

void DebugDraw::render(const Mat4& view_projection) {
    glUseProgram(shader_);
    glUniformMatrix4fv(mvp_loc_, 1, GL_FALSE, glm::value_ptr(view_projection));
    glBindVertexArray(vao_);

    // Draw thin lines (width 1.0)
    if (!line_vertices_.empty()) {
        glLineWidth(1.0f);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(line_vertices_.size() * sizeof(Vertex)),
                     line_vertices_.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(line_vertices_.size()));
    }

    // Draw thick lines
    if (!thick_line_vertices_.empty()) {
        glLineWidth(thick_line_width_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(thick_line_vertices_.size() * sizeof(Vertex)),
                     thick_line_vertices_.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(thick_line_vertices_.size()));
        glLineWidth(1.0f);
    }

    // Draw points (re-upload as individual vertices)
    if (!points_.empty()) {
        std::vector<Vertex> point_verts;
        point_verts.reserve(points_.size());
        for (const auto& p : points_) {
            point_verts.push_back({p.position, p.color});
        }

        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(point_verts.size() * sizeof(Vertex)),
                     point_verts.data(), GL_DYNAMIC_DRAW);

        // Draw each point with its own size
        GLsizei idx = 0;
        for (const auto& p : points_) {
            glPointSize(p.size);
            glDrawArrays(GL_POINTS, idx, 1);
            ++idx;
        }
    }

    glBindVertexArray(0);
    glUseProgram(0);

    line_vertices_.clear();
    thick_line_vertices_.clear();
    points_.clear();
}

} // namespace kaizen
