# Animation System — Technical Design

## What This System Does

Two human figures grapple in real-time. The game layer produces a stream of discrete state changes (position transitions, control variable updates, technique attempts). The animation system's job is to make those state changes look like two people doing jiu jitsu — continuous, physical, reactive, fluid.

The game layer is authoritative. The animation system never drives gameplay; it visualizes it. If the animation can't perfectly represent something, gameplay still resolves correctly and the animation catches up.

## Why Not Canned Animations

The position graph has hundreds of positions with thousands of transitions between them. Authoring unique animations for each is combinatorially explosive and produces worse results than procedural generation because:

- **Start state varies.** A tripod sweep from a neutral DLR looks different than one from a DLR where the top player was heavily loaded forward. Canned animations can't account for the starting configuration — they teleport the characters to the animation's expected start pose, which looks janky.
- **Transitions should flow.** The system must produce motion that starts from wherever the bodies currently are. No teleporting. No blending between unrelated poses. Every frame follows continuously from the last.
- **Sub-position variation.** Within a single position, control variable inputs produce visible changes (posture breaking, weight shifting, hooks degrading). These are continuous, not discrete states with distinct animations.
- **Two bodies in contact.** Both characters are always constraining each other. Animating them independently and hoping they line up doesn't work.

## Core Idea: Constraint-Driven Incremental Pose Solving

### The Constraint Graph

At any moment, the two bodies form a **constraint graph**:

- **Nodes:** ~54 bones (two 27-bone humanoid skeletons)
- **Tree edges:** parent-child relationships within each skeleton (shoulder→elbow→wrist, etc.)
- **Constraint edges:** connections between the two bodies — grips, hooks, weight contact, ground contact

A position in the game graph maps to a set of active constraints between the two bodies. Closed guard means the bottom player's legs wrap the top player's waist (hook constraints on both legs connecting to the opponent's torso), plus whatever grips are active. The constraints fully define the spatial relationship between the two bodies.

The solver does not see "two separate characters." It sees one system of ~54 bones connected by joint limits and inter-body constraints.

### Persistent Pose, Incremental Solving

The system maintains a **persistent pose** for both bodies across frames. Each frame, the solver makes small adjustments to satisfy the current constraint set. This is the opposite of the sandbox approach (rebuild from rest every frame) and gives us:

- **Continuous motion.** Each frame is a small perturbation of the last. No discontinuities.
- **Temporal coherence.** Joint rotations (including twist) carry over. The solver doesn't need to reconstruct orientation information from scratch.
- **Natural transitions.** When constraints change (technique executes, grip established), the solver incrementally adapts from the current pose. The motion emerges from the constraint change — no authored transition animation needed.
- **Stability.** Small input = small output. The pose doesn't flicker or oscillate because the solver is always starting from a nearly-valid solution.

### What Happens Each Frame

```
1. Start from previous frame's pose (both bodies, ~54 bones)
2. Update constraint set from game layer:
   - Hard constraints: grips hold, hooks hold, ground contact
   - Target constraints: game layer directing limbs (e.g., "breakfall with left arm")
   - Intent attractors: soft bias for unconstrained limbs (see Intent System below)
3. Run iterative solver:
   - Satisfy hard constraints (priority 1)
   - Satisfy target constraints (priority 2)
   - Minimize energy toward intent attractors (priority 3, uses remaining DOFs)
   - Enforce joint limits throughout
4. Apply temporal damping for stability
5. Output: joint angles for all ~54 bones → world transforms → render
```

## Techniques as Constraint Timelines

A technique is not an animation. It is a **timed sequence of constraint changes** that the solver realizes as motion.

Example — tripod sweep:

```
Phase 1 (0.0s - 0.3s): Setup
  - Foot-on-hip constraint activates (foot_l → opponent hip region)
  - Ankle grip constraint tightens (hand_r → opponent ankle)

Phase 2 (0.3s - 0.8s): Execution
  - Foot-on-hip target drives outward (extend hip)
  - Ankle grip target pulls toward player (pull ankle)
  - DLR hook drives laterally (off-balance direction)

Phase 3 (0.8s - 1.2s): Resolution (success path)
  - Opponent's center of mass passes tipping point
  - Opponent balance breaks → game layer triggers fall reaction
  - Top player intent: breakfall with near arm, frame with far arm
  - Bottom player releases constraints, begins stand-up

Phase 3 (0.8s - 1.2s): Resolution (failure path)
  - Opponent resists, constraints insufficient to break balance
  - Bottom player constraints release, return to guard recovery
  - Control variables update (fatigue spent, position may degrade)
```

The technique data authors ~5-8 constraint events. The solver generates all the actual motion — spine bending, weight shifting, compensatory limb movement, defensive reactions. The same technique looks different depending on the starting configuration because the solver adapts from wherever the bodies currently are.

### What Gets Authored Per Position

Each position in the game graph needs animation-relevant data:

```yaml
animation:
  constraints:
    # What physically defines this position
    - type: hook
      from: { body: bottom, bone: lower_leg_l }
      to: { body: top, bone: upper_leg_r, region: inner_thigh }
    - type: grip
      from: { body: bottom, bone: hand_r }
      to: { body: top, bone: foot_r, region: heel }
    - type: grip
      from: { body: bottom, bone: hand_l }
      to: { body: top, bone: lower_leg_r, region: knee }
    - type: ground_contact
      body: bottom
      bones: [pelvis, hand_l]  # seated, posting with left hand

  # What the unconstrained limbs should be doing (see Intent System)
  intent:
    bottom:
      free_limbs:
        # No truly free limbs in this position — both hands gripping, both legs engaged
    top:
      free_limbs:
        hand_l: { intent: balance, bias: forward_post }
        hand_r: { intent: balance, bias: lateral_post }
        # Both hands free to post for balance since bottom has leg grips not arm grips
```

### What Gets Authored Per Technique

```yaml
technique:
  id: tripod_sweep
  phases:
    - name: setup
      duration: [0.2, 0.4]  # range for timing variation
      constraints_add:
        - type: contact
          from: { body: bottom, bone: foot_r }
          to: { body: top, bone: pelvis, region: hip }
      targets:
        - bone: { body: bottom, bone: foot_r }
          drive: { direction: forward_up, strength: 0.6 }

    - name: execution
      duration: [0.3, 0.6]
      targets:
        - bone: { body: bottom, bone: foot_r }
          drive: { direction: extend_hip, strength: 1.0 }
        - bone: { body: bottom, bone: hand_r }
          drive: { direction: pull_toward_self, strength: 0.8 }

    - name: resolution_success
      duration: [0.4, 0.8]
      constraints_remove:
        - { from: bottom.hand_r, to: top.foot_r }  # release ankle
      opponent_reaction: fall_backward
      # Game layer takes over — new position established

    - name: resolution_failure
      duration: [0.3, 0.5]
      constraints_remove:
        - { from: bottom.foot_r, to: top.pelvis }  # foot pushed off hip
      # Return to position, constraints revert
```

This is intentionally sparse. The solver fills in the motion from these keyframes. The authored data says "push here, pull there, in this sequence" and the solver makes two bodies move accordingly.

## The Intent System

Every limb has a purpose, even when it's not directly constrained. The intent system prevents unconstrained limbs from going dead or floppy.

### Why This Matters

In real jiu jitsu, there are no idle limbs. The top player in someone's guard has both hands doing something — posting for balance, framing against sweeps, setting up grips, blocking hip movement. When a technique attempt fails, the defending player doesn't go limp — they immediately reframe, repost, readjust. This constant purposefulness is what makes grappling look like grappling rather than ragdoll physics.

### Intent Categories

These are behavioral modes that define what a limb's soft target should be:

- **Balance / posting** — limb biases toward a load-bearing position. For arms: hands toward the mat, elbows slightly bent, ready to catch weight. For legs: feet under hips, knees tracking over toes. This is the default for standing or top-position limbs.
- **Framing / defensive structure** — limb biases toward blocking the opponent's advancement. Forearms across the body, elbows tight, creating distance. Default for bottom-position free arms.
- **Guard retention** — legs bias toward staying between the two bodies, knees to chest, feet on hips/biceps. Default for bottom-position free legs.
- **Recovery** — limb returning to a neutral ready position after being used. Brief transition intent after a constraint releases.
- **Technique preparation** — limb drifting toward the next grip or hook target in anticipation. The game layer can signal "this limb is about to be used for X" and the intent system pre-positions it.

### How Intent Feeds the Solver

Intent produces **soft target poses** per limb — not precise positions, but joint angle ranges that represent "this arm should look roughly like it's posting." These are the lowest-priority input to the solver: hard constraints come first, then explicit targets, and intent fills in whatever degrees of freedom remain.

The intent bias is expressed as a rest-pose attractor in the solver's null space. When a limb has no hard constraints or explicit targets, it drifts toward its intent pose. When it does have constraints, the intent is overridden but can still influence remaining DOFs (e.g., a gripping arm's elbow position isn't fully determined by the grip — intent can bias whether the elbow is high or low).

Intent changes when:
- Position changes (new position has different default intents)
- Game layer signals a reaction (breakfall, frame, scramble)
- A constraint is added or removed (limb role changes)

These intent changes should have brief blending periods so the limb doesn't snap to a new pose.

## Solver Architecture

### Why Not FABRIK

The sandbox uses a tree FABRIK solver. FABRIK is fast and intuitive for open-chain endpoint targeting, but it has fundamental limitations for our use case:

- **No twist solving.** FABRIK solves positions. The rotation recovery step uses minimum-rotation alignment, which never produces axial twist. Shoulder internal/external rotation, hip rotation — critical for jiu jitsu poses — can't be solved.
- **No orientation targets.** Grips need palm direction, not just hand position. FABRIK only solves positions.
- **No closed loops.** Grips between two bodies create closed kinematic chains. FABRIK is an open-chain solver. Extensions exist but they're bolted on.
- **Position-level, not joint-level.** FABRIK works in Cartesian space and converts back to joint angles. This makes constraint enforcement awkward — joint limits are naturally expressed in joint angle space.

The sandbox FABRIK solver remains useful for interactive bone dragging in debug/test contexts. It's not the production animation solver.

### Jacobian-Based Solver with Prioritized Tasks

The production solver operates in **joint angle space** and uses the **Jacobian** to map between joint angle changes and constraint satisfaction.

**Why Jacobian-based:**
- Works directly with joint angles — twist is just another DOF, handled naturally
- Handles orientation targets (6-DOF end effectors: position + rotation)
- Handles closed loops (constraints between any two bones, not just parent-child)
- Prioritized task framework: solve hard constraints first, use remaining DOFs for soft targets
- Incremental by nature: computes Δθ (small joint angle changes) each iteration
- Well-documented in robotics literature, extensively used in game animation middleware

**Solver loop (per frame):**

```
Input: current joint angles θ, constraint set C, intent targets

For each iteration (typically 3-10 per frame):
  1. Compute forward kinematics: θ → world positions/orientations
  2. Compute constraint errors: Δx = target - current for each constraint
  3. Build Jacobian J: how each joint angle affects each constraint
  4. Priority level 1 — hard constraints (grips, hooks, ground):
     Δθ₁ = J₁⁺ · Δx₁                    (pseudoinverse solve)
     Project to null space of J₁
  5. Priority level 2 — target constraints (technique drives):
     Δθ₂ = (J₂N₁)⁺ · (Δx₂ - J₂Δθ₁)    (solve in null space of level 1)
     Project to null space of J₁ and J₂
  6. Priority level 3 — intent attractors:
     Δθ₃ = (J₃N₁₂)⁺ · (Δx₃ - J₃(Δθ₁+Δθ₂))
  7. Apply: θ += Δθ₁ + Δθ₂ + Δθ₃
  8. Clamp joint limits
  9. Check convergence
```

The null-space projection is what gives us the priority system: hard constraints are satisfied exactly (or as close as joint limits allow), and remaining freedom is used for lower-priority goals. A grip stays locked even if the intent system wants the arm somewhere else.

**Damped least squares** (Levenberg-Marquardt) prevents instability near singularities — when a limb is fully extended and the Jacobian becomes ill-conditioned, damping keeps the solver from producing wild joint angle changes.

### Constraint Types

| Type | DOFs | Description |
|------|------|-------------|
| Position pin | 3 | Bone world position = target position. Used for grips, hooks, ground contact. |
| Orientation pin | 3 | Bone world orientation = target orientation. Used for palm direction on grips. |
| Full pin | 6 | Position + orientation. Used when both matter (e.g., RNC grip lock). |
| Distance | 1 | Distance between two bones = target. Used for maintaining contact without full position lock. |
| Look-at | 2 | Bone axis points toward target. Used for head tracking. |
| Soft target | 3-6 | Same as above but with spring-like behavior — pulls toward target with configurable strength rather than hard constraint. Used for intent attractors. |

### Two Bodies as One System

The solver doesn't run twice (once per body). It runs once over the combined system:

- Joint angle vector θ contains all DOFs from both bodies (~120-160 total)
- Jacobian J maps all joints to all constraints
- A grip constraint between body A's hand and body B's wrist generates Jacobian columns for joints in both arms — moving A's shoulder OR B's shoulder can satisfy the grip

This means the solver naturally finds solutions where both bodies adjust to satisfy shared constraints. If a grip is pulling two hands together, the solver might move one arm more than the other based on which has more available freedom — exactly how real grappling works.

The two bodies' root translations are DOFs in the system too. If all the constraints pull body A toward body B, body A's root can translate to help satisfy them. Root translation is weighted lower than joint rotations to prevent the bodies from sliding around excessively — positional changes should come primarily from the game layer (position transitions), with the solver making small adjustments.

## Control Variable Visualization

While in a position, control variable inputs need visible expression on the characters. When the bottom player extends the DLR hook (building backward off-balance), the opponent's posture should visibly shift. When the top player drives forward to kill the hook, their weight should visibly move into the bottom player.

This is handled through the constraint/target system:

- Each control variable input has associated **animation effects** — soft targets that shift body positions to reflect the input. Extending the hook adds a soft target pulling the opponent's upper body backward. Driving forward adds a soft target pushing the opponent's weight center forward.
- These effects are proportional to the control variable's current value, not the input itself. As off-balance accumulates, the visible postural shift increases.
- The intent system adjusts in response — as the top player gets loaded backward, their free arms shift to more aggressive posting to reflect the fight for balance.

This is lower priority than the core constraint solver but critical for the game to feel readable. The player needs to see their inputs affecting the opponent's body.

## Integration with Game Layer

### Data Flow

```
Game Layer                          Animation Layer
───────────                         ───────────────
Position graph state        ──→     Active constraint set
Control variable values     ──→     Visualization soft targets
Technique attempt           ──→     Constraint timeline (phases)
Technique resolution        ──→     Outcome constraints + reactions
AI/player reactions         ──→     Intent changes (breakfall, frame, etc.)
                            ←──     (Nothing — animation doesn't drive gameplay)
```

The animation layer is a pure consumer of game state. It never reports back "the hand reached the target" or "the sweep animation finished" to trigger game logic. The game layer runs on its own tick and the animation layer visualizes whatever state the game layer produces.

### Timing

The game simulation runs at a **fixed timestep** (e.g., 60 Hz). The pose solver runs once per sim tick. Rendering interpolates between sim frames for smooth display at the monitor's refresh rate.

Technique timelines are measured in sim ticks, not wall time. The durations in the technique data (e.g., "setup phase: 0.2-0.4s") translate to tick counts at the sim rate.

## Performance

The math is small:

- ~54 bones, ~120-160 DOFs
- ~5-15 constraints per frame (grips, hooks, ground, targets)
- Jacobian matrix: ~30-90 rows × 120-160 columns
- 3-10 solver iterations per frame
- Total: well under 1ms per frame on any modern hardware

The Jacobian computation (step 3) is the most expensive per-iteration operation — computing how each DOF affects each constraint requires forward kinematics partial derivatives. For our bone count this is trivial. A naive implementation will be fast enough; optimization (sparse Jacobian exploitation, analytical derivatives vs. finite differences) is available if needed but unlikely to be.

## What the Sandbox FABRIK Solver Becomes

The existing FABRIK solver in `src/core/src/ik.cpp` remains as a sandbox tool for interactive skeleton testing. It's useful for:

- Verifying skeleton construction and joint limits
- Testing rendering (bone visualization, debug draw)
- Quick iteration on bone proportions and constraint ranges

It does not evolve into the production solver. The production solver is a separate system built on different foundations (Jacobian, joint-space, prioritized tasks, two-body).

## Open Questions

### Intent Authoring Granularity

How specific do intent definitions need to be per position? Options:
- **Coarse:** A few categories (posting, framing, guard retention) with per-category default poses. Positions just assign categories to limbs.
- **Fine:** Per-position authored joint angle targets for each free limb. More work but more control over the look.
- **Hybrid:** Coarse categories with per-position overrides where the default doesn't look right.

The hybrid approach is likely correct — start coarse, add overrides as needed during visual polish. The categories themselves will emerge during development as common patterns become apparent.

### Technique Phase Interpolation

How should the solver transition between technique phases? Options:
- **Hard switch:** at phase boundary, constraints change instantly. The solver smooths it out incrementally over subsequent frames. Simplest, may produce visible pops for large constraint changes.
- **Blended switch:** constraints from the ending phase fade out while constraints from the new phase fade in over a brief overlap window (e.g., 3-5 frames). Smoother, more parameters to tune.

Hard switch with solver damping is the starting point. Add blending if the pops are visible.

### Ground Interaction Model

How do characters interact with the ground? Options:
- **Explicit ground constraints:** positions author which bones contact the ground. The solver pins them.
- **Ground plane enforcement:** a global constraint that prevents any bone from penetrating the floor, with friction.
- **Both:** authored ground contacts for stable positions (knees on mat, feet planted) plus global floor enforcement as a safety net.

The "both" approach is likely needed. Authored contacts give stable, correct ground interaction in known positions. Global floor enforcement prevents limbs from going through the mat during transitions.

### Root Translation Weighting

How much should the solver move character roots vs. adjusting joint angles? If a grip pulls two characters together, should one character slide toward the other, or should both extend their arms? In real grappling this depends on base and balance — the person with better base moves less. This needs a tunable weight per-body that can change with game state (balance, stance, position).

### Collision Between Bodies

The constraint system handles intentional contact (grips, hooks). What about unintentional contact — preventing the characters' torsos from overlapping, limbs from passing through each other? Options:
- **Ignore it:** rely on stylization (see game-design.md section 8) to make clipping invisible. Cheapest, works for most cases.
- **Capsule collision:** approximate each limb with a capsule, add soft separation constraints when capsules overlap. Moderate cost, prevents gross interpenetration.
- **Selective collision:** only enforce non-penetration for large body segments (torso, thighs) where clipping is most visible. Ignore hands/forearms where contact is expected.

Start with "ignore it" + stylization. Add selective capsule collision if clipping becomes distracting during playtesting.
