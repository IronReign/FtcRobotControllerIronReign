# Competition Setup Notes — March 2026

Changes since last competition. Read this before match day.

---

## Init Loop Controls (Updated)

| Input | Action |
|---|---|
| B | Select **Red** alliance |
| X | Select **Blue** alliance |
| A | Set starting position: **Audience wall** |
| Y | Set starting position: **Goal wall** |
| Back | Set starting position: **Unknown** |
| Right Trigger (hold) | Cycle game state |
| D-pad Right | Cycle abort-after-rows: ALL / 0 / 1 / 2 |
| **Left Bumper** | **NEW** — Toggle intake for preloading balls |
| **Guide (Home)** | **CHANGED** — Turret encoder calibration (see below) |
| Left Stick Y | Drive forward/backward |
| Right Stick X | Turn |

### Preloading Balls in Init Loop

Left bumper now runs the intake during init loop. Same behavior as teleop — press to start loading, press again to stop. Use this to preload balls consistently before the match instead of hand-feeding.

### Turret Calibration (Two-Phase)

The turret encoder zeros at power-on, wherever the turret is physically pointing. If it powered on at a bad angle, the turret will think "forward" is wrong and aim incorrectly.

**How to tell it's wrong:** During init loop, the turret should hold steady pointing forward. If it's visibly off-center or fighting to hold a wrong position, calibrate it.

**How to fix it:**
1. Press **Guide (Home)** once — long rumble. The turret motor goes limp.
2. Manually rotate the turret to its center/forward position by hand.
3. Press **Guide (Home)** again — short rumble. Encoder zeros at the current position. The turret re-engages.

You only need to do this if the turret looks wrong at startup. Normal power-on with the turret centered does not need calibration.

---

## Alliance + Position Selection

**Order no longer matters.** You can select alliance and position in any order — changing alliance now re-applies the starting pose automatically. Previously, pressing alliance after position left the pose stale.

Always verify on Dashboard that the robot's position dot is in the correct spot on the field after selecting both.

---

## Audience Start (Major Changes)

### What Changed

Previously, audience start skipped straight to launching with no targeting. The turret often couldn't acquire the goal, and shots went wide.

Now:
1. **Turret auto-seeks the goal** using odometry-based bearing from match start. It rotates toward the goal immediately without needing vision.
2. **Targeting step before first launch** — the robot waits (up to 2 seconds) for turret lock before firing preloads. If vision acquires the target, it refines. If not, odometry bearing is accurate enough for preloads.
3. **Fixed flywheel speed at audience range** — vision distance is unreliable at long range, so the robot uses a preset speed (MIN_LAUNCH_SPEED_AUDIENCE) instead of vision-computed distance. This happens automatically when the robot's field X position is past 32 inches (audience side). No driver action needed.

### What the Driver Sees

- After pressing Start on audience, the turret will swing toward the goal, the flywheel spins up, and launching begins once the turret is on target.
- LEDs will show green (balls loaded), then may flash white (intermittent vision lock) — this is normal at audience range.
- The robot will fire even with intermittent vision, using odometry aiming as fallback.

---

## Shooting Changes (Both Starts)

### Pulsed Firing with Speed Check

The feed-pause-feed cycle between balls now waits for the flywheel to recover speed before feeding the next ball. Previously it was purely timer-based, so the third ball often lacked speed.

- Goal range: 300ms minimum pause, then waits for speed recovery
- Audience range: 500ms minimum pause, then waits for speed recovery
- Hard timeout (1.5s) is the safety backstop if the flywheel never recovers

**What the driver sees:** Slightly longer pauses between balls at audience range, but all three balls should now have consistent speed.

### Lock Quality (Improved for Audience)

The turret's "ready to fire" signal now survives brief vision dropouts (under 0.3 seconds). Previously, a single frame of lost vision reset the entire lock timer, causing long delays before the launcher would fire. This was the main cause of the "turret looks on target but won't fire" behavior at audience range.

---

## Teleop Changes

### Flywheel Speed Switching

The flywheel speed now switches automatically based on field position:
- **Robot X > 32 inches** (audience side): Uses preset audience speed
- **Robot X < 32 inches** (goal side): Uses vision-computed distance

This means the driver does NOT need to manually switch between shootShort / shootLong (D-pad Down / D-pad Up) in most cases. Those manual overrides still work if needed.

### Button Map (Teleop)

| Input | Action | Notes |
|---|---|---|
| Left Bumper | Toggle intake | Same as before |
| Right Bumper | Launch all | Same as before |
| X | Spin up flywheel | Same as before |
| Y | Set turret to tracking | Same as before |
| A | Toggle slow mode | Same as before |
| B | Emergency stop | Same as before |
| D-pad Up | Shoot long preset | Manual override |
| D-pad Down | Shoot short preset | Manual override |
| D-pad Left | Simple intake on | No auto-stop |
| D-pad Right | Eject balls | Two-phase eject |
| Start | Cycle Limelight tilt | Same as before |
| Back | Apply relocalization | Rumble confirms |

---

## Quick Reference: What to Check Before Each Match

1. **Power on** with turret pointed forward
2. Select **alliance** (B=Red, X=Blue)
3. Select **starting position** (A=Audience, Y=Goal)
4. Verify pose on Dashboard field overlay
5. If turret looks off-center: **Guide** to release, center by hand, **Guide** again
6. **Left Bumper** to preload balls via intake
7. Press **Start** when ready

---

## Dashboard Parameters Worth Knowing

| Parameter | Section | Default | What it does |
|---|---|---|---|
| CENTERING_TIMEOUT_SECONDS | Lebot2_Autonomous | 2.0 | Max wait for turret lock before launching (audience) |
| PRESET_SPEED_X_THRESHOLD | Lebot2_Launcher | 32 | Field X boundary for switching to preset speed |
| MIN_LAUNCH_SPEED_AUDIENCE | Lebot2_Launcher | 1350 | Preset flywheel speed at audience range |
| PULSE_PAUSE_MS | Lebot2_Launcher | 300 | Min recovery pause between balls (goal) |
| PULSE_PAUSE_MS_FAR | Lebot2_Launcher | 500 | Min recovery pause between balls (audience) |
| HOLD_POSITION_S | Lebot2_Turret | 0.3 | Vision dropout grace period before resetting lock |
| GOOD_ENOUGH_DEG | Lebot2_Turret | 3.0 | Turret oscillation threshold for "good enough" lock |
