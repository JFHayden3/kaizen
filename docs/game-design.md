# Kaizen — Game Design Document

## Vision

A single-player jiu jitsu game that captures the experience of *training and improving* at jiu jitsu over time. Not a fighting game with a jiu jitsu skin — a game where grappling knowledge, strategy, and fatigue management drive the outcome. The player's character grows in skill and knowledge; the player themselves develops strategic intuition for when and how to attack.

The initial scope is **no-gi jiu jitsu exclusively**. The no-gi technique set is significantly smaller and more manageable for initial development. Gi-specific grips, techniques, and positions can be explored as a future content expansion.

The core tension: techniques are available to attempt at any time, but *setting them up* — through pressure, grip fighting, off-balancing, and fatigue accumulation — is what makes them actually work. A berimbolo from a neutral de la riva is a gamble. A berimbolo after you've loaded their weight forward, killed their posting hand, and drained their legs is inevitable.

---

## Core Systems

### 1. Position Graph

The grappling state is modeled as a **directed graph of discrete positions** with known transitions between them.

**Positions** are the primary nodes. Every distinct grappling configuration is a position — closed guard, half guard, mount, de la riva, de la riva w/ sleeve grip, armbar, rear naked choke w/ arm under chin, etc. There is no engine-level distinction between a "major position" and a "grip variation." Each position has:

- A set of **available techniques** (sweeps, submissions, passes, escapes)
- A set of **available pressures** (directional forces, weight distribution changes)
- Defined **top/bottom/neutral** role context
- Per-position fatigue drain rates (some positions are more exhausting to maintain)
- Optional **fatal** flag (see below)

The difference between "de la riva" and "de la riva w/ sleeve grip" is captured entirely in the **transition properties** between them — low fatigue cost, low counter risk, fast transition speed. The difference between "closed guard" and "mount" is also just transition properties — high cost, high risk, requires technique. The engine treats both uniformly; the UI can present nearby low-cost positions as grip/configuration options distinct from major techniques (see section 7).

**Transitions** are the edges — each technique, when attempted, maps to one or more outcome positions:

- **Success** → attacker's target position (e.g., sweep lands → top position)
- **Failure** → position resets or degrades (e.g., failed sweep → guard recovered, or guard partially passed)
- **Counter** → defender's advantageous position (e.g., failed sweep → guard passed)

**Fatal positions** are submission positions that end the match once solidly established. Submissions are not a separate system — they are positions in the graph like any other, with their own pressures and counters. The progression into a submission is a chain of position transitions like anything else (e.g., back control → rear naked choke w/ arm over chin → rear naked choke w/ arm under chin → choke locked w/ figure-four grip). What makes fatal positions special is only the end condition: once a fatal position reaches a **consolidation threshold** (a function of attacker control, defender fatigue, and positional stability), the match ends in a tap.

The path *to* a fatal position is handled entirely by the standard technique/counter system. The defender has every opportunity to counter during transitions — and even once in a submission position, escape techniques exist as normal counters (see Defensive Explosions in section 4).

All positions, techniques, transitions, and their relationships are **hand-authored**. This is not a procedural or physics-derived system — it encodes actual jiu jitsu knowledge.

### 2. Control Variables & Player Inputs

This is the core mechanical skill layer — the thing the player is *doing* moment-to-moment between technique attempts.

Each position defines a set of **control variables** — contested, continuous state values that both players are trying to influence in their favor. Control variables are the setup layer: they feed into technique success rates, gate technique availability, and represent the micro-battles (grip fighting, off-balancing, angle creation, hook maintenance) that make techniques actually work.

**Types of control variables:**

Control variables share a common interface — they have a value, both players have inputs that affect them, and they feed into technique modifiers — but they differ in their **behavior properties**:

- **Off-balance** (directional) — represents instability in a given direction. *Decays* over time when not actively fed. Decay rate is influenced by opponent fatigue (a tired opponent recovers balance more slowly). Multiple directional axes can exist per position. Example: backward off-balance in standing DLR, built by hamstring extension.
- **Grip control** — represents whether a grip is established and maintained. *Sticky* — once established, it persists until actively broken by the opponent. Costs fatigue to establish, low or no cost to maintain, costs the opponent fatigue to break. Changes available techniques and inputs. Example: free leg control in standing DLR, top player's collar tie from clinch.
- **Positional advantage** — represents slow, structural changes to the position. *Contested* — one player builds it, the other actively suppresses or reverses it. Doesn't decay on its own but can be undone. Example: angle creation in DLR (bottom builds, top circles to suppress), posture breakdown in closed guard (bottom pulls, top frames to posture up).
- **Structural integrity** — represents the health of a position-defining feature. *Degraded* by the opponent, *maintained* passively or actively by the holder. When it drops below a threshold, it may force a position transition (hook gets killed → position degrades to open guard). Example: DLR hook integrity under forward pressure.

These categories are not rigid engine types — they're behavioral patterns expressed through the properties authors set on each variable (decay rate, contestation model, persistence, thresholds). The engine evaluates all control variables uniformly.

**Player inputs:**

Players influence control variables through stick and trigger inputs:

- **Left analog stick** — primary input. Contextual to current position. From closed guard bottom, pulling the stick down = breaking posture. From standing DLR top, driving stick forward = killing the hook.
- **Triggers + stick combinations** — when a position has many input options, trigger modifiers expand the input space. E.g., LT + stick = hip-level actions, RT + stick = upper-body actions.
- A single input can affect **multiple control variables simultaneously** — driving forward into a DLR hook degrades hook integrity AND drains opponent leg fatigue, but also potentially feeds forward off-balance that the opponent can exploit.
- The **dynamic UI** (see section 7) is critical here — it communicates what each input *does* to which control variables, so the player isn't guessing.

**Interaction and contestation:**

Both players are applying inputs simultaneously, and the engine resolves their interactions each tick:

- **Opposing inputs** can partially cancel (both pushing off-balance in opposite directions) or create exploitable dynamics (top player drives forward to kill hook, bottom player redirects that energy into forward off-balance).
- **Reaction provocation** — sustained pressure on one control variable may cause the AI opponent to commit to countering it, which opens opportunities on *other* variables. Better opponents read this and react appropriately; worse opponents over-commit and get exploited.
- **All inputs cost fatigue.** Influencing control variables drains attacker fatigue. Contesting/maintaining against an opponent's inputs drains defender fatigue. The exchange rate depends on position, attributes, and current fatigue levels — good technique is *efficient* (costs you less than it costs them).

### 3. Technique Execution

At any time, the player can attempt a technique from their currently available set (determined by current position + unlocked knowledge).

**Attempt flow:**

1. Player selects a technique (button press mapped to the on-screen technique menu)
2. System evaluates **success probability** based on:
   - Base technique difficulty
   - Current **control variable state** — each technique defines which control variables modify its success rate and in what direction (e.g., tripod sweep benefits from backward off-balance; deep DLR entry requires angle creation above a threshold)
   - Opponent fatigue (overall + relevant limbs)
   - Attacker fatigue (overall + relevant limbs)
   - Attacker's technique proficiency level
   - Opponent's relevant defensive proficiency
   - Opponent's current state (are they mid-reaction? mid-technique? committed to contesting a control variable?)
   - Attribute matchup (size, strength, flexibility differentials)
3. System resolves outcome: **success**, **failure**, or **counter**
4. Position transitions accordingly
5. Brief transition plays out (see Animation section) — player is not in control during the transition but can see the result

**Partial success** should exist as a concept — a sweep attempt that doesn't fully land but advances position (e.g., intended sweep to mount, partial success = lands in half guard top). This prevents the system from feeling too binary.

### 4. Counter System

The opponent is always a threat. While the player is setting up offense, the opponent is doing the same — applying their own pressure, working their own setups, and attempting their own techniques.

**Opponent technique attempts** surface to the player as a **counter opportunity** with a **time pressure element**:

- The player sees the incoming technique (signaled through UI + animation cues)
- A window opens to select an appropriate counter
- Available counters depend on the player's **counter knowledge** for that specific technique
- Speed of recognition and response matters — earlier counters are more effective, later counters may only partially defend
- Failing to counter (or not knowing a counter) means the opponent's technique resolves against you using the same success probability system

This is where player *skill* matters most directly — reading the opponent's attacks, recognizing what's coming, and selecting the right counter under time pressure. But it's bounded by character knowledge: you can't counter what your character hasn't learned to counter.

**Defensive explosions:** Many techniques — sweeps, guard passes, submissions — have a critical moment during execution where a well-timed, high-energy defensive response can defeat the technique. These are modeled as a specific class of counter:

- Triggered during a **transition window** — the brief period while a technique is resolving, before the new position is consolidated
- Cost **significant stamina** — these are explosive, last-resort efforts. A player relying on defensive explosions will gas out fast.
- Effectiveness depends on **specific counter knowledge** — not just "explode out" but knowing *how* to explode. A hitchhiker escape timed to the moment an armbar grip breaks. A granby roll as a back-take attempt clears the hips. The right escape at the right moment.
- Earlier timing = more effective, lower stamina cost. Late explosions may still partially work but cost more and are less likely to succeed.
- This gives even fatal positions (submissions) a defense window — you can be *in* an armbar and still escape if you have the knowledge, the timing, and the stamina. But each failed escape attempt drains you further, making the next one harder. The submission becomes inevitable through attrition, not a binary lock.

**Counter cascade:** A countered technique can itself be countered (if the attacker knows the re-counter), creating scramble sequences. The depth of these chains is bounded by both players' knowledge graphs.

### 5. Fatigue System

Fatigue operates on two levels:

**Overall fatigue** — a global stamina meter. Depletes with all actions. When low:
- Technique success rates drop
- Pressure output decreases
- Counter windows shrink
- Recovery between positions slows
- Partially recovers during neutral/rest positions at a rate influenced by conditioning attribute

**Per-limb fatigue** — tracked for major limb groups (left arm, right arm, left leg, right leg, core/hips). Specific techniques and pressures tax specific limbs.
- A collar grip fight drains arms
- Playing de la riva drains the hook-side leg
- Bridging from bottom drains core
- When a limb is fatigued, techniques relying on that limb are less effective AND more susceptible to submissions targeting that limb (a fatigued arm is easier to armbar)
- Limb fatigue recovers more slowly than overall fatigue

**Fatigue asymmetry** is a core strategic element:
- Bigger/stronger opponents drain your fatigue faster through pressure but may gas out if you can keep them working
- Efficient technique creates favorable fatigue trades
- Positional dominance typically means more efficient fatigue exchange (mount top costs less energy than mount bottom escape attempts)
- Stalling in a position is a valid but potentially costly strategy (referee intervention in competition modes)

### 6. AI Opponent System

Opponents operate on the same underlying system as the player — they exist in the position graph, have available techniques and pressures, and manage fatigue.

**Opponent behavior is driven by:**

- **Technique library** — what techniques the opponent knows (mirrors the player's unlock system). A white belt opponent has fewer options than a black belt.
- **Proficiency levels** — how well the opponent executes known techniques. Affects their success rates.
- **Strategy tendencies** — behavioral weights that define their style:
  - Aggression: how frequently they attempt techniques vs. building setups
  - Patience: how long they'll work a setup before committing
  - Preferred positions: some opponents actively seek specific positions (guard players vs. top pressure players)
  - Preferred techniques: weighted likelihood toward their A-game
- **Reaction quality** — how appropriately they respond to player pressure. Lower skill = more predictable/exploitable reactions. Higher skill = technically correct reactions, occasional mixups.
- **Attribute profile** — size, strength, flexibility, cardio. Creates meaningful physical matchups.

**Skill tiers are expressed through all of the above simultaneously** — a tough opponent isn't just "higher numbers," they have more techniques, better reactions, smarter aggression timing, and better fatigue efficiency. A beginner opponent might know a lot of techniques but have poor timing and predictable reactions.

---

## Presentation

### 7. Dynamic UI

The UI must solve a hard problem: communicating a complex, context-dependent state space without overwhelming the player.

**On-screen elements (active during gameplay):**

- **Position indicator** — current position name. Always visible.
- **Technique menu** — available techniques from current position. Mapped to face buttons or d-pad. Techniques should display contextual effectiveness indicators (color-coded or icon-based) based on current setup quality. This menu updates live as position changes and as setup conditions shift. Nearby low-cost transitions (grip changes, minor adjustments) should be visually distinguished from major techniques — same underlying system, but the UI should make it clear which transitions are "adjustments" vs. "commitments."
- **Pressure guide** — a contextual overlay showing what each stick direction does in the current position. Could be a radial diagram around the character or a HUD element. This is essential — without it, stick-based pressure is unreadable. Fades or minimizes as the player develops familiarity (could be a player-controlled detail level).
- **Fatigue displays** — overall stamina bar + limb fatigue indicators for both players. Needs to be glanceable, not noisy. Could use a body silhouette with limb heat-mapping.
- **Off-balance indicator** — directional indicator showing accumulated off-balance on the opponent. Could be integrated into the pressure guide.
- **Opponent telegraph** — visual cues when the opponent is building toward a technique attempt. Becomes the counter-timing signal.

**Design challenge:** This is a lot of information. The UI will need careful layering — critical info always visible, secondary info surfaced contextually, deep info available on demand. Playtesting will drive iteration here heavily.

### 8. Animation System

The gameplay logic is discrete and hand-authored. The animation system's job is to **visually represent** these discrete states convincingly in 3D — it does not drive gameplay.

**Approach: Procedural animation over authored skeletons (Cairn model)**

Rather than authoring unique animations for every technique (combinatorially explosive and brittle), use a hybrid system:

- **Pose targets** — each position has a defined target pose for both players (skeletal poses representing correct body positioning for that grappling state). These are hand-authored to look correct.
- **Procedural interpolation** — transitions between poses are driven procedurally. IK (inverse kinematics) handles limb placement. Physics-informed blending handles weight and momentum. The system interpolates between the "before" and "after" poses of a technique with procedurally generated in-betweens.
- **Pressure visualization** — while the player applies pressure, subtle procedural animation shows the effect: opponent's posture breaking, weight shifting, limbs adjusting. This is cosmetic but critical for readability — the player needs to *see* that their pressure is working.
- **Authored keyframes for signature moments** — certain high-impact techniques (submission locks, dramatic sweeps) may warrant hand-authored animation sequences or keyframe guides that the procedural system follows more closely. These are the "payoff" moments.
- **Contact management** — a constraint system ensuring bodies stay in contact where they should (grips maintained, hooks in place, weight on chest). This is where physics-based jank typically emerges, so careful constraint tuning and authored contact point definitions per position are essential.

**The key principle:** Gameplay state is *always* authoritative. The animation system visualizes it. If the animation system can't perfectly represent a transition, the gameplay still resolves correctly and the animation catches up. The visual can be slightly imperfect — the game logic cannot be.

### 9. Camera System

Camera is **game-controlled** — no manual camera rotation during gameplay (hands are busy with pressure inputs and technique selection).

**Camera types** (player can cycle between preferences):

- **Broadcast** — simulates a competition broadcast angle. Side view, slightly elevated. Good readability for horizontal guard play and scrambles.
- **Over-the-shoulder** — closer, behind the player's character. More immersive, good sense of pressure and space. May occlude some positions.
- **Top-down** — overhead view. Best for reading the position graph spatially. Least cinematic but most informationally clear.
- **Dynamic** — game selects camera angle based on current position and action. Could be disorienting but most cinematic.

Camera will need per-position hints to avoid bad angles (e.g., camera shouldn't be looking through a body during back mount). This is a system that will need heavy iteration.

---

## Design Principles

1. **Knowledge over reflexes.** The character's knowledge (unlocked techniques, proficiency levels) is the primary driver of what's possible. Player skill is in *strategic decision-making* — reading the opponent, managing fatigue, choosing when to commit — not in execution precision.

2. **Setup over spam.** Attempting techniques without setup should feel like a gamble. Patient, intelligent setup play should feel powerful and rewarding. The system must make the difference in success rates *dramatic enough* that players feel the value of pressure and patience.

3. **Authenticity over spectacle.** Every technique, transition, and positional interaction should reflect actual jiu jitsu. Players with real grappling knowledge should recognize what's happening and feel represented. Players without it should *learn* something about how grappling actually works.

4. **Readable complexity.** The underlying system is complex — dozens of positions, hundreds of techniques, multi-dimensional fatigue, directional off-balance. But the moment-to-moment experience must be *clear*: I'm here, I can do these things, this is what's working, this is what's threatening me.

---

## Open Questions & Future Work

These are explicitly deferred from the core gameplay design but noted for future development.

- **Progression system** — how the player unlocks techniques, improves proficiency, develops attributes. The outer loop. Could model an actual training journey (drilling, sparring, competition). Deferred until core gameplay is proven.
- **Match/round structure** — competition rules (IBJJF points, ADCC, submission-only), training rolls, specific scenarios. The core system should be agnostic to scoring — different rule sets layer on top. Deferred.
- **Curriculum/campaign** — a structured single-player experience. Training under instructors, competing in tournaments, developing a personal game. Deferred.
- **Gi expansion** — adding gi-specific grips, positions, and techniques as a future content layer on top of the no-gi foundation. Deferred.
- **Weight classes and body types** — how size/strength/flexibility differentials are modeled and balanced. Noted in attribute system but detailed design deferred.
- **Technique discovery** — can the player discover technique chains or setups that weren't explicitly authored? Or is the system strictly bounded by authored content? Philosophical design question.
- **Animation pipeline** — detailed technical approach for the procedural animation system. Needs prototyping. Deferred.
- **Engine/platform selection** — no engine commitment yet. Deferred until core systems are prototyped.
