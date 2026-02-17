#include "kaizen/platform/window.h"
#include "kaizen/game/match_engine.h"

#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

int main(int /*argc*/, char* /*argv*/[]) {
    kaizen::Window window({"Kaizen Sandbox", 1280, 720});

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForOpenGL(window.sdl_window(), window.gl_context());
    ImGui_ImplOpenGL3_Init("#version 330");

    kaizen::MatchEngine engine;

    while (!window.should_close()) {
        window.poll_events([](const SDL_Event& event) {
            ImGui_ImplSDL2_ProcessEvent(&event);
        });

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Kaizen Sandbox");
        ImGui::Text("Match engine sandbox — ready for prototyping.");
        ImGui::End();

        ImGui::Render();
        int w, h;
        SDL_GL_GetDrawableSize(window.sdl_window(), &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.12f, 0.12f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        window.swap();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    return 0;
}
