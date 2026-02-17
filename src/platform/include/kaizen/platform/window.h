#pragma once

#include <SDL.h>
#include <functional>
#include <string>

namespace kaizen {

struct WindowConfig {
    std::string title = "Kaizen";
    int width  = 1280;
    int height = 720;
};

class Window {
public:
    explicit Window(const WindowConfig& config = {});
    ~Window();

    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    SDL_Window*   sdl_window() const { return window_; }
    SDL_GLContext  gl_context() const { return gl_context_; }
    bool           should_close() const { return should_close_; }

    using EventHandler = std::function<void(const SDL_Event&)>;

    void poll_events(EventHandler handler = nullptr);
    void swap();

private:
    SDL_Window*   window_     = nullptr;
    SDL_GLContext  gl_context_ = nullptr;
    bool           should_close_ = false;
};

} // namespace kaizen
