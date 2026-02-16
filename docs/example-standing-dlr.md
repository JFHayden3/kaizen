# Example Position Walkthrough: Standing De La Riva

Working example for developing the core data model and engine logic.

---

## Position Data Model

A position represents a **two-player state** — both players' options are defined together because they're inseparable. You can't describe "standing DLR bottom" without also describing what the top player can do.

### Position: Standing DLR — Heel Grip + Knee Control

```yaml
position:
  id: "standing_dlr_heel_knee"
  display:
    name: "De La Riva"
    detail: "Heel Grip + Knee Control"  # UI can show this as secondary info
    group: "de_la_riva"                 # for UI grouping of related positions

  # Per-position passive fatigue drain — just being here costs something
  fatigue_drain:
    bottom:
      overall: 0.02    # per second, normalized 0-1 scale
      legs: 0.04       # DLR hook is tiring on the legs
    top:
      overall: 0.01    # standing is less tiring than playing guard
      legs: 0.02

  # --- CONTROL VARIABLES ---
  # These are the contestable state values that define the "setup" for this position.
  # Both players' inputs push these in their favor. Technique modifiers reference them.
  control_variables:
    - id: "off_balance_backward"
      type: "off_balance"
      display: "Weight Over Heels"
      initial: 0.0
      range: [0.0, 1.0]
      decay: 0.08               # decays per second when not fed
      decay_fatigue_scaling: true  # decays slower when opponent is fatigued

    - id: "off_balance_forward"
      type: "off_balance"
      display: "Weight Forward"
      initial: 0.0
      range: [0.0, 1.0]
      decay: 0.08

    - id: "off_balance_lateral"
      type: "off_balance"
      display: "Lateral Off-Balance"
      initial: 0.0
      range: [0.0, 1.0]
      decay: 0.06

    - id: "angle"
      type: "positional_advantage"
      display: "Back Exposure"
      initial: 0.0
      range: [0.0, 1.0]
      decay: 0.0                # does NOT decay — must be actively reversed
      # bottom builds it, top suppresses/reverses it

    - id: "hook_integrity"
      type: "structural_integrity"
      display: "DLR Hook"
      initial: 0.8              # starts strong — you have the position
      range: [0.0, 1.0]
      decay: 0.0                # doesn't decay on its own
      threshold_effect:
        below: 0.2
        result: "force_transition"
        target: "open_guard_seated"  # hook dies → position collapses

    - id: "free_leg_control"
      type: "grip_control"
      display: "Free Leg"
      initial: 0.0              # starts uncontrolled (bottom player's favor)
      range: [0.0, 1.0]
      decay: 0.0                # sticky — stays until actively broken
      # 0.0 = bottom has full use of free leg
      # 1.0 = top has complete control of the free leg
      suppression_threshold: 0.6  # above this, suppresses bottom inputs that use free leg

  # --- BOTTOM PLAYER (guard player) ---
  bottom:

    inputs:
      - id: "hamstring_extension"
        display: "Extend Hook"
        input: { stick: "back" }
        effects:
          control: { off_balance_backward: +0.3 }
          opponent_fatigue: { legs: 0.03 }
          self_fatigue: { legs: 0.04, overall: 0.01 }

      - id: "free_leg_hip_push"
        display: "Hip Push"
        input: { stick: "forward", trigger: "LT" }
        requires: { free_leg_control: { max: 0.6 } }  # suppressed when leg is controlled
        effects:
          control: { off_balance_lateral: +0.2, angle: +0.25 }
          opponent_fatigue: { core: 0.02 }
          self_fatigue: { legs: 0.03, overall: 0.01 }

      - id: "lateral_pull"
        display: "Pull Sideways"
        input: { stick: "left" }
        effects:
          control: { off_balance_lateral: +0.25, angle: +0.3 }
          self_fatigue: { arms: 0.02, overall: 0.01 }

      - id: "butt_scoop_hook"
        display: "Scoop Under"
        input: { stick: "forward" }
        requires: { free_leg_control: { max: 0.6 } }  # needs the free leg
        effects:
          control: { off_balance_forward: +0.35 }
          opponent_fatigue: { core: 0.02, legs: 0.02 }
          self_fatigue: { legs: 0.05, overall: 0.02 }

      - id: "fight_free_leg"
        display: "Free Leg"
        input: { stick: "down", trigger: "LT" }
        effects:
          control: { free_leg_control: -0.3 }  # fights to reclaim the leg
          self_fatigue: { legs: 0.03, overall: 0.01 }

    transitions:
      # --- Low-cost position changes (grip switches) ---
      - id: "to_scoop_grip_dlr"
        display: "Switch to Scoop Grip"
        target: "standing_dlr_scoop_grip"
        input: { button: "LB" }
        fatigue_cost: { arms: 0.01 }
        counter_risk: 0.05
        speed: "fast"
        # control variable carryover: angle, off-balance carry over to the new position
        # if the new position has matching control variable IDs, values transfer

      - id: "disengage_to_open_guard"
        display: "Disengage"
        target: "open_guard_seated"
        input: { button: "LB", modifier: "down" }
        fatigue_cost: { overall: 0.03 }
        counter_risk: 0.1
        speed: "fast"

      # --- Major techniques ---
      - id: "tripod_sweep"
        display: "Tripod Sweep"
        input: { button: "X" }
        outcomes:
          success:
            target: "top_open_guard_standing"
            probability_base: 0.25
          partial:
            target: "top_open_guard_scramble"
            probability_base: 0.2
          failure:
            target: "standing_dlr_heel_knee"
            probability_base: 0.4
          counter:
            target: "guard_passed_half"
            probability_base: 0.15
        modifiers:
          - condition: { off_balance_backward: { min: 0.5 } }
            adjust: { success: +0.25, partial: +0.1, failure: -0.2, counter: -0.15 }
          - condition: { opponent_fatigue_legs: { min: 0.4 } }
            adjust: { success: +0.1, failure: -0.1 }
          - condition: { self_fatigue_legs: { min: 0.6 } }
            adjust: { success: -0.15, counter: +0.1 }
          - condition: { hook_integrity: { min: 0.5 } }  # need a solid hook to sweep from
            adjust: { success: +0.1 }
        fatigue_cost: { legs: 0.08, core: 0.05, overall: 0.05 }

      - id: "deep_dlr_entry"
        display: "Deep DLR Hook"
        input: { button: "Y" }
        outcomes:
          success:
            target: "deep_dlr_heel_knee"
            probability_base: 0.15
          failure:
            target: "standing_dlr_heel_knee"
            probability_base: 0.6
          counter:
            target: "open_guard_passed_hq"
            probability_base: 0.25
        modifiers:
          - condition: { angle: { min: 0.6 } }           # the key one
            adjust: { success: +0.45, failure: -0.25, counter: -0.2 }
          - condition: { off_balance_lateral: { min: 0.3 } }
            adjust: { success: +0.1, counter: -0.1 }
        fatigue_cost: { legs: 0.06, overall: 0.03 }

      - id: "waiter_guard_entry"
        display: "Waiter Guard"
        input: { button: "A" }
        outcomes:
          success:
            target: "waiter_guard"
            probability_base: 0.1
          failure:
            target: "standing_dlr_heel_knee"
            probability_base: 0.5
          counter:
            target: "guard_passed_side_control"
            probability_base: 0.4
        modifiers:
          - condition: { off_balance_forward: { min: 0.6 } }
            adjust: { success: +0.4, failure: -0.15, counter: -0.25 }
        fatigue_cost: { legs: 0.07, core: 0.04, overall: 0.04 }

      - id: "single_leg_x_entry"
        display: "Single Leg X"
        input: { button: "B" }
        outcomes:
          success:
            target: "single_leg_x"
            probability_base: 0.2
          failure:
            target: "standing_dlr_heel_knee"
            probability_base: 0.5
          counter:
            target: "guard_passed_half"
            probability_base: 0.3
        modifiers:
          - condition: { off_balance_forward: { min: 0.4 } }
            adjust: { success: +0.3, failure: -0.15, counter: -0.15 }
          - condition: { opponent_fatigue_legs: { min: 0.3 } }
            adjust: { success: +0.1 }
        fatigue_cost: { legs: 0.06, overall: 0.03 }

  # --- TOP PLAYER (passer) ---
  top:

    inputs:
      - id: "forward_pressure_kill_hook"
        display: "Drive Into Hook"
        input: { stick: "forward" }
        effects:
          control: { hook_integrity: -0.15 }
          opponent_fatigue: { legs: 0.05 }
          self_fatigue: { legs: 0.02, overall: 0.01 }

      - id: "diagonal_pressure_pin_hip"
        display: "Pin Hip"
        input: { stick: "forward_left" }
        effects:
          control: { angle: -0.3 }            # reverses angle creation
          opponent_fatigue: { core: 0.02 }
          self_fatigue: { legs: 0.02, overall: 0.01 }

      - id: "circular_movement_stay_square"
        display: "Circle Away"
        input: { stick: "right" }
        effects:
          control: { angle: -0.4, off_balance_lateral: -0.3 }  # strong angle counter + resets lateral kuzushi
          self_fatigue: { overall: 0.02 }

      - id: "control_free_leg"
        display: "Control Free Leg"
        input: { stick: "down", trigger: "RT" }
        effects:
          control: { free_leg_control: +0.3 }  # establish/maintain control
          opponent_fatigue: { legs: 0.03 }
          self_fatigue: { arms: 0.02, overall: 0.01 }

    transitions:
      - id: "knee_cut"
        display: "Knee Cut Pass"
        input: { button: "X" }
        outcomes:
          success:
            target: "side_control"
            probability_base: 0.15
          partial:
            target: "half_guard_top_knee_cut"
            probability_base: 0.25
          failure:
            target: "standing_dlr_heel_knee"
            probability_base: 0.4
          counter:
            target: "deep_dlr_heel_knee"
            probability_base: 0.2
        modifiers:
          - condition: { hook_integrity: { max: 0.4 } }   # hook is compromised
            adjust: { success: +0.2, partial: +0.1, failure: -0.15, counter: -0.15 }
          - condition: { angle: { max: 0.2 } }            # hip is pinned/square
            adjust: { success: +0.15, counter: -0.1 }
        fatigue_cost: { legs: 0.06, core: 0.04, overall: 0.04 }

      - id: "leg_drag"
        display: "Leg Drag"
        input: { button: "Y" }
        outcomes:
          success:
            target: "leg_drag_passing"
            probability_base: 0.15
          failure:
            target: "standing_dlr_heel_knee"
            probability_base: 0.55
          counter:
            target: "single_leg_x"
            probability_base: 0.3
        modifiers:
          - condition: { free_leg_control: { min: 0.6 } }   # have the leg to drag
            adjust: { success: +0.25, failure: -0.15, counter: -0.1 }
          - condition: { opponent_fatigue_legs: { min: 0.5 } }
            adjust: { success: +0.1 }
        fatigue_cost: { arms: 0.05, overall: 0.03 }

      - id: "stepover_to_hq"
        display: "Step Over to HQ"
        input: { button: "A" }
        outcomes:
          success:
            target: "headquarters"
            probability_base: 0.1
          failure:
            target: "standing_dlr_heel_knee"
            probability_base: 0.5
          counter:
            target: "waiter_guard"
            probability_base: 0.4
        modifiers:
          - condition: { hook_integrity: { max: 0.3 } }   # hook is nearly dead
            adjust: { success: +0.35, failure: -0.2, counter: -0.15 }
          - condition: { free_leg_control: { min: 0.5 } }  # controlling the step-over leg
            adjust: { success: +0.15, counter: -0.1 }
        fatigue_cost: { legs: 0.05, overall: 0.03 }

      - id: "disengage_to_open"
        display: "Disengage"
        input: { button: "LB" }
        outcomes:
          success:
            target: "open_guard_standing_top"
            probability_base: 0.6
          failure:
            target: "standing_dlr_heel_knee"
            probability_base: 0.3
          counter:
            target: "single_leg_x"
            probability_base: 0.1
        modifiers:
          - condition: { hook_integrity: { max: 0.4 } }
            adjust: { success: +0.3, failure: -0.25, counter: -0.05 }
        fatigue_cost: { overall: 0.02 }
```

---

## Engine State

At any given tick, the engine tracks:

```yaml
state:
  position: "standing_dlr_heel_knee"
  tick: 4823

  bottom_player:
    fatigue:
      overall: 0.35
      left_arm: 0.20
      right_arm: 0.15
      left_leg: 0.55    # DLR hook leg — been working hard
      right_leg: 0.30
      core: 0.25
    active_input: "hamstring_extension"  # currently holding stick back
    input_duration: 1.8                  # seconds on current input

  top_player:                            # AI-controlled
    fatigue:
      overall: 0.25
      left_arm: 0.15
      right_arm: 0.10
      left_leg: 0.30
      right_leg: 0.20
      core: 0.15
    active_input: "forward_pressure_kill_hook"
    input_duration: 0.6

  # Control variable current values — the "setup" ledger
  control:
    off_balance_backward: 0.42    # bottom has been extending the hamstring
    off_balance_forward: 0.0
    off_balance_lateral: 0.15
    angle: 0.18                   # some angle work but top has been circling
    hook_integrity: 0.55          # started at 0.8, top has been working it down
    free_leg_control: 0.25        # top has partial control, not yet suppressing

  # Opponent action state
  opponent_action: "idle"   # or "attempting_technique", "recovering"
```

---

## Gameplay Walkthrough

### Scenario: Bottom player works a tripod sweep setup

**Tick 0 — Position entered.** Both players arrive in `standing_dlr_heel_knee`. Control variables initialize to their defined `initial` values (off-balance at 0, angle at 0, hook_integrity at 0.8, free_leg_control at 0). The UI shows:

- Position label: "De La Riva — Heel Grip + Knee Control"
- Technique menu: Tripod Sweep (dim), Deep DLR (dim), Waiter Guard (dim), Single Leg X (dim) — all dim because no setup has been done, base probabilities are low
- Input guide: radial display showing stick directions and what control variables they affect
- Grip transitions: Scoop Grip DLR, Disengage — shown as quick-switch options

**Seconds 0-3 — Bottom player extends the hook (stick back).** Engine applies `hamstring_extension` input each tick:

- `off_balance_backward` climbs: 0.0 → 0.12 → 0.22 → 0.30
- Bottom player's `left_leg` fatigue ticks up (0.04/s from the input + 0.04/s passive DLR drain)
- Top player's `legs` fatigue ticks up slightly (0.03/s)
- Tripod Sweep indicator starts brightening — its modifier references `off_balance_backward`

**Second 1.5 — AI opponent starts driving forward** (`forward_pressure_kill_hook`). Now both players are applying inputs simultaneously:

- `hook_integrity` starts dropping (top's input applies -0.15/s)
- Bottom player's `left_leg` fatigue accelerates — they're extending while the opponent is compressing. This is an inefficient exchange for the bottom player.
- The engine resolves **simultaneous control variable effects**: the top player's forward drive doesn't directly oppose `off_balance_backward` (it targets `hook_integrity`), so the off-balance continues to build — but the bottom player is paying more fatigue to maintain it against the hook pressure.
- The bottom player now faces a decision: keep extending (costly, hook getting killed) or switch inputs.

**Second 3 — Bottom player switches to butt scoop (stick forward).** This redirects the attack:

- `off_balance_backward` starts decaying (no input feeding it, decay rate 0.08/s)
- `off_balance_forward` starts climbing (+0.35/s from the butt scoop). The opponent is *already driving forward* to kill the hook — the engine should recognize that inputs pushing the same control variable in the same direction compound. The top player's forward drive is now inadvertently helping load `off_balance_forward`.
- This is a key strategic moment — the top player's commitment to killing the hook is now feeding the bottom player's forward off-balance setup. The AI needs to recognize this and adjust.

**Second 4 — AI recognizes forward loading, switches to circular movement.** The AI's reaction quality determines how fast this adjustment happens. A lower-skilled opponent might keep driving forward for another second or two, feeding the setup. A high-level opponent switches almost immediately.

- Circling applies `angle: -0.4` and `off_balance_lateral: -0.3`, but crucially it *stops* the hook degradation and the inadvertent forward loading
- `off_balance_forward` accumulation slows (only the bottom player's butt scoop feeding it now)
- But the bottom player got ~1 second of compounding forward load

**Second 5.5 — `off_balance_forward` at 0.45, `off_balance_backward` decayed to 0.15.** The bottom player could attempt a waiter guard entry (`off_balance_forward` modifier kicks in at 0.6 — not there yet) or switch back to hamstring extension to rebuild backward off-balance for the tripod.

Bottom player switches back to hamstring extension. They're working a **pressure cycle** — forward, backward, forward — each time the opponent adjusts, they switch directions and carry residual off-balance from prior efforts.

**Second 8 — `off_balance_backward` at 0.55, opponent legs fatigued at 0.35.** Tripod sweep indicator is bright. The modifier at `off_balance_backward >= 0.5` gives `+0.25 success`. The opponent's leg fatigue at 0.35 gives `+0.1`. The `hook_integrity` is still at 0.6 (above 0.5 threshold) giving another `+0.1`. Net probability: `0.25 (base) + 0.25 + 0.1 + 0.1 = 0.70 success`. Much better than the raw 0.25.

**Player presses X — Tripod Sweep.**

Engine resolves:
1. Evaluate modified probabilities: ~70% success, ~10% partial, ~12% fail, ~8% counter
2. Roll outcome (weighted random with the modified distribution)
3. Let's say: **success**
4. Position transitions to `top_open_guard_standing`
5. Transition animation plays — procedural blend from DLR pose to standing-over-swept-opponent pose
6. Control variables reset to the new position's `initial` values — the setup ledger is fresh
7. Fatigue carries over — the bottom player's legs are tired from all that DLR work

### Scenario: Top player disrupts and passes

Same starting point, but this time the AI opponent is aggressive and skilled.

**Seconds 0-2 — Bottom player extends hook.** Same as before, backward off-balance building.

**Second 1 — AI immediately drives forward AND controls the free leg** (alternating inputs). `hook_integrity` starts dropping. `free_leg_control` starts climbing. Once `free_leg_control` crosses 0.6 (the `suppression_threshold`), the bottom player's `free_leg_hip_push` and `butt_scoop_hook` inputs become unavailable — they literally can't use those moves while the leg is controlled. Forward off-balance options are now limited to indirect methods.

**Second 3 — `hook_integrity` at 0.45, `free_leg_control` at 0.65, bottom player's left leg fatigue at 0.50.** The bottom player's DLR hook is getting cooked and their free leg is controlled. Their input menu has visibly shrunk — hip push and butt scoop are grayed out. They could spend an input on `fight_free_leg` to reclaim it, but that means they're not building any offensive setup. Tripod sweep is still available but the self leg fatigue modifier is starting to penalize it.

**Second 4 — AI attempts knee cut.** The engine evaluates from the top player's perspective:
- `hook_integrity` at 0.4 (below the 0.4 max threshold) → `+0.2 success, +0.1 partial`
- `angle` still at ~0.05 (below the 0.2 max threshold) → `+0.15 success`
- Modified success: `0.15 + 0.2 + 0.15 = 0.50`

**Counter window opens for the bottom player.** UI signals incoming knee cut. If the bottom player knows a counter to knee cut from this position (e.g., re-pummel to deep half, or hip escape to reguard), they have a ~2 second window to select it. Earlier selection → more effective.

**Player selects "reguard to half guard" counter at 1.2 seconds** (moderate timing).

Engine resolves the interaction:
- Knee cut success probability: 0.50
- Counter effectiveness: moderate timing reduces knee cut success by ~0.20
- Adjusted: 0.30 success for the passer
- Outcome roll: **partial success** — position transitions to `half_guard_top_knee_cut` rather than full side control
- Bottom player is now in half guard — not ideal, but they defended the pass to side control

---

## Design Questions Surfaced

### Control Variable Carryover on Position Transitions

When a position transition happens, control variables reset to the new position's `initial` values. But some transitions are between closely related positions (e.g., `standing_dlr_heel_knee` → `standing_dlr_scoop_grip`) where it would feel wrong for off-balance work to vanish. If both positions define a control variable with the same `id`, should the value carry over? This needs a clear rule — probably an explicit `carryover: true/false` flag per control variable, or per transition.

### Simultaneous Input Resolution

Both players apply inputs every tick. When their inputs affect the same control variable (e.g., bottom builds angle, top suppresses angle), the engine needs a clear resolution model. Simple additive (bottom's +0.3 and top's -0.4 = net -0.1/s) is the obvious starting point, but there may be cases where interactions are non-linear — e.g., driving forward into someone who's scooping under you should compound the forward off-balance more than additively. Needs prototyping.

### Suppression vs. Probability Penalty

The `suppression_threshold` on `free_leg_control` creates a hard cutoff — below 0.6, inputs are available; above 0.6, they're gone. An alternative is a smooth penalty curve where controlled inputs become increasingly ineffective rather than disappearing. The hard cutoff is more readable (input is clearly available or not) but the smooth curve might feel less jarring. Could also do both: smooth penalty on effectiveness as the variable climbs, with full suppression at a high threshold.

### AI Input Selection

The AI needs to choose which input to apply each tick. This is a non-trivial decision — the top player in this example has four input options that serve different strategic purposes (kill hook, pin hip, circle away, control free leg) and can only do one at a time. The AI needs to evaluate which control variables are most threatening, which of its own techniques are closest to viable, and how to allocate its attention across multiple contested fronts. This is likely the hardest AI problem in the system.
