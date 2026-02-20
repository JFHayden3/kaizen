# Content Pipeline

How technique and position data gets authored and imported into the engine. This is a constraint-driven system — we author sparse constraint events, not full animations.

## Prior Art

Every shipped grappling game (WWE 2K, AKI/No Mercy, EA UFC) uses **paired pre-baked animations**: both characters' motions are captured/animated together as a unit, and the grappling system selects which paired animation to play. WWE 2K22 captured 4,800+ animations via mocap. AKI's N64 games were entirely hand-animated. EA UFC uses a position state machine with pre-captured transitions.

Our system is fundamentally different. A technique is ~5-8 constraint events with timing, not a full motion sequence. The solver generates all actual motion. This trades the massive content volume problem (thousands of paired animations) for an authoring tool problem (making it fast to specify constraint events).

No direct prior art exists for a constraint-driven grappling content pipeline in shipped games. The closest analogue is Overgrowth (Wolfire), which used procedural IK for combat, but only for brief striking contacts — not sustained grappling.

## Position Representation

Nothing in jiu jitsu is truly fixed. A "position" is a region in configuration space, not a single pose. The representation has three layers:

### Invariants

Constraints that **define** the position. If any break, you've transitioned to a different position node in the graph. These specify topological relationships and regions, not exact bone positions.

Example: closed guard's invariant is "legs locked around the waist." The contact can slide within the waist region. Open the legs → different position (open guard).

```yaml
invariants:
  - type: contact_wrap
    body: bottom
    bones: [lower_leg_l, lower_leg_r]
    target_body: top
    target_region: waist
    break_triggers: [open_guard, half_guard]
```

### Dimensions

Continuous parameters that vary **within** the position. These map directly to control variables. Both players contest these in real time.

Example: in closed guard, "posture" ranges from broken down to fully postured. Both players push on it. The current value drives the solver via animation effects (spine rotation range).

```yaml
dimensions:
  posture:
    range: [0, 1]             # 0 = broken down, 1 = fully postured
    default: 0.4
    decay: 0.1                # drifts toward broken posture
    animation_effect:
      bone: top.spine_upper
      rotation_range: [-0.5, 0.3]  # radians, maps to posture value
  hip_angle:
    range: [-1, 1]            # bottom player's angle off-center
    default: 0
    decay: 0.2
    animation_effect:
      bone: bottom.hip
      rotation_range: [-0.4, 0.4]
```

Gameplay loop: players push on dimensions until a threshold enables a transition edge (sweep attempt) or breaks an invariant (guard opens).

### Attractors

Where limbs settle without input. The intent system's defaults per position. Fill remaining DOFs after invariants and dimensions are satisfied.

```yaml
attractors:
  bottom:
    arms: framing              # forearms across body
  top:
    arms: posting              # hands on mat/hips
```

### Mapping to Solver Priorities

This three-layer model maps directly to the prioritized null-space projection solver:

- Priority 1 (hard constraints) = invariants
- Priority 2 (technique drives) = dimension manipulation by the game layer
- Priority 3 (intent) = attractors

## Contact Points Along Bones

Contacts in grappling aren't always at joint endpoints. "Forearm across the face" targets the middle of the forearm, not the elbow or wrist. The IK system supports parametric bone targets:

```
target position = lerp(pos[bone], pos[child_bone], t)
```

Where `t ∈ [0, 1]` specifies position along the bone segment. The Jacobian columns are a linear interpolation of the two endpoint Jacobians — no new solver machinery needed.

Examples: shin across hip (t=0.5 on lower_leg), blade of wrist (t=0.9 on forearm), center of forearm to face (t=0.5 on forearm).

## Video Capture Setup

Reference video for the authoring tool. Not full automated mocap — provides a rough starting point that the author refines.

### Camera Rig

- **Two horizontal cameras at ~90°** (front + side of mat area) — resolves depth ambiguity from a single view
- **One overhead camera** (phone on ceiling mount or tall tripod boom) — critical for ground positions where the top player occludes the bottom from horizontal views
- All three cameras cover the same capture area on the mat

### Synchronization

Frame-exact sync isn't required — constraint events operate at phase timescales (tenths of seconds). A clap or visible event at the start of each technique rep syncs the views in post. At 30fps, even crude sync is within ~33ms.

### Camera Calibration

One-time setup per session (cameras stay put after calibration):

1. **Per-camera intrinsics**: Hold a printed checkerboard (A3/poster size) in front of each camera individually at various angles, ~10-15 shots. OpenCV computes focal length and lens distortion.
2. **Shared extrinsics**: Tape markers on the mat at known measured positions (e.g. 4 colored tape pieces at corners of a 2m square), visible from all cameras. Since the real-world coordinates are known, each camera's position/orientation relative to the mat can be computed. This establishes a shared coordinate system.

### Capture Workflow

1. Calibrate cameras (one-time per session)
2. Perform technique reps with sync clap before each
3. Standing work (clinch, takedowns) — front + side cameras are sufficient
4. Ground work (guard, mount, side control) — overhead camera becomes essential

## Processing Pipeline

1. **Pose estimation** per view independently (MediaPipe, ViTPose, etc.) — will be noisy/broken for grappling due to occlusion, that's expected
2. **Triangulate** joints visible in 2+ views — gives reasonable 3D positions for unoccluded joints
3. **Flag** low-confidence joints (occluded in most views) for manual correction
4. **Load** rough 3D skeleton into authoring tool alongside video reference panels
5. **Author** manually corrects bad joints, marks contact events, sets phase timing, defines constraint types

The cameras provide a skeleton that's ~60-70% correct. Author expertise fills in the rest. This is a massive speedup over posing from scratch without trying to solve fully automated grappling pose estimation (which is an unsolved problem due to extreme occlusion).

## Authoring Tool (Future)

Side-by-side editor:

- **Left panel**: synced video views (front, side, overhead), scrubbable frame-by-frame
- **Right panel**: two IK skeletons with the constraint solver running live
- **Workflow**: pose key moments to match video reference, mark contacts, set phase timing, define constraint types and regions
- **Output**: technique YAML matching the engine's data model (see `example-standing-dlr.md` for format reference)

The solver shows what the technique will look like in-engine in real time as the author works. Export directly to the technique data format.
