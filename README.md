# kaizen

Single-player no-gi jiu jitsu game. Custom C++ engine focused on training and improvement progression.

## Building

Requires CMake 3.24+ and a C++20 compiler. All other dependencies are fetched automatically.

```bash
cmake -B build
cmake --build build
```

## Running

```bash
./build/bin/kaizen_sandbox   # ImGui sandbox for prototyping game systems
./build/bin/kaizen            # Game executable (placeholder)
ctest --test-dir build        # Run tests
```

## Build options

Pass these to `cmake -B build`:

| Option | Default | Description |
|---|---|---|
| `KAIZEN_BUILD_SANDBOX` | `ON` | Build the ImGui sandbox |
| `KAIZEN_BUILD_TESTS` | `ON` | Build tests |
| `KAIZEN_BUILD_TOOLS` | `OFF` | Build content pipeline tools |

## Project structure

```
src/
  core/       Engine core — types, data loading, RNG (links yaml-cpp)
  game/       Game systems — positions, match engine (NO rendering deps)
  platform/   SDL2 + OpenGL window/context abstraction
  render/     Rendering (future)

apps/
  sandbox/    ImGui prototype sandbox for developing game systems
  game/       Game executable
  tools/      Content pipeline tools (future)

tests/        Catch2 tests against game systems (headless, no GPU)

data/
  positions/  YAML position data files

docs/         Design documents
```

The key architectural invariant: `game/` never depends on rendering. Game systems can be developed, tested, and iterated on entirely through the headless test suite and the ImGui sandbox.
