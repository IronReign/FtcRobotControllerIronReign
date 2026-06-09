# Lebot2 Driver's Manual

All controls use **Gamepad 1**. Buttons are edge-detected (press, not hold) unless noted.

---

## Init Mode (Before Match Start)

Controls active while the OpMode is initialized but not yet started.

| Input | Action |
|---|---|
| Right Trigger (hold) | Cycle game state: Autonomous / Tele-Op / Test |
| X | Select **Blue** alliance |
| B | Select **Red** alliance |
| A | Set starting position: **Audience wall** |
| Y | Set starting position: **Goal wall** |
| Back | Set starting position: **Unknown** (for teleop testing) |
| **D-pad Right** | **Selective abort**: cycle how many ball rows to collect before parking — **ALL → 0 → 1 → 2 → ALL** (see Autonomous Coordination below). Current value shows in init telemetry as "Abort After". |
| **Left Bumper** | **Toggle intake** to preload balls during setup |
| Guide (Home) | Turret calibration — **currently unreliable, see note below** |
| Left Stick Y | Drive forward/backward (for pre-match positioning) |
| Right Stick X | Turn left/right |

**Starting positions** set the Pinpoint odometry's initial pose. Choose the one matching where the robot is physically placed:
- **Audience**: Touching audience wall, can see goal AprilTag at range
- **Goal**: Touching goal wall (default for competition)
- **Unknown**: Position unknown, relies on vision to localize

> **Turret calibration:** the Guide-button two-phase calibration is **currently broken — don't rely on it.** Instead, **physically rotate the turret so the tic marks line up** (turret pointing straight forward) **before or during init**. The encoder zeros to that aligned position, so getting the tic marks aligned at startup is the calibration.

---

## Tele-Op Mode

### Driving

| Input | Action |
|---|---|
| Left Stick Y | Throttle (forward/backward) |
| Right Stick X | Turn (dampened by slow mode setting) |

Drive input is always active. If a mission or trajectory is running, significant joystick input (>10% deflection) automatically aborts it and returns control to the driver.

Normal turn dampening is 90%. Slow mode (toggle with **A**) reduces it to 40%.

### Controls

| Input | Action |
|---|---|
| **Left Bumper** | **Toggle intake** (LOAD_ALL): runs the intake until the loader is full, then auto-stops. Press again to stop manually. |
| **Right Bumper** | **Launch all** loaded balls in sequence (spin up with **X** first; intake assists during the volley). |
| **X** | **Spin up flywheel** to target speed. |
| **Y** | **Turret tracking on** — turret seeks the goal (vision, then odometry). Also applies a relocalization. |
| **B** | **Emergency stop** — cancels launcher, intake, belt; returns to manual control. |
| **A** | **Toggle slow mode** (precise turning). |
| **Start** | **Toggle turret lock** — LOCKED (held forward) vs tracking. Use to park a misbehaving turret. |
| **Back** | **Apply vision pose correction** (relocalize) when a tag is visible. Long rumble = applied, short = no botpose. |
| **Guide (Home)** | **Reset drive encoders.** |
| **D-pad Down** | **Shoot-short preset** — near-range speed/feed (use when vision distance is unreliable). |
| **D-pad Up** | **Shoot-long preset** — far/audience-range speed/feed. |
| **D-pad Left** | **Simple intake on** (continuous, no auto-stop). |
| **D-pad Right** | **Eject** the top ball (brief forward pulse + reverse reseat). |

### Launch Sequence Detail

The full launch cycle when you press **Right Bumper**:

1. Flywheel must already be spinning (press **X** first) — and the turret should be locked on the goal (white turret LEDs).
2. Right Bumper triggers LAUNCH_ALL: it waits for the flywheel at speed + turret ready, then fires.
3. **Pulsed firing** feeds the balls one at a time (feed / pause / feed) so the flywheel recovers speed between balls for consistent distance.
4. After all balls are out, the flywheel drops to a **warm idle** (stays spinning for a fast next volley).

If something goes wrong, press **B** to emergency stop everything.

---

## LED Status Indicators

The SparkFun LED stick (10 LEDs) is split into three zones so loader and turret status are visible at the same time. Read it left-to-right by zone:

| LEDs | Zone | Meaning |
|---|---|---|
| **Ends (1 & 10)** | Alliance | Always your alliance color (red / blue). If these are the wrong color, fix alliance selection in Init. |
| **Inner-left (2–5)** | Loader | **Off** = empty · **Amber** = has balls (1–2) · **Green** = full (3, ready) · **Flashing green** = overfull (4th ball, auto-eject clearing it) |
| **Inner-right (6–9)** | Turret / Vision | **Off** = not seeking · **Yellow** = aiming by odometry (no tag yet) · **Cyan** = tracking the tag · **White** = locked & ready to fire · **Pulsing white** = firing |

Quick reads during a match:
- **Green loader + white turret** = loaded and locked, take the shot.
- **Green loader stays green** = chamber is full; intake has auto-stopped.
- **Yellow turret** = it knows roughly where the goal is from odometry but hasn't seen the tag — usually resolves to cyan/white once the camera picks up the AprilTag.
- **Flashing green** = a 4th ball is being ejected; let it clear.

The flash/pulse rates (`OVERFULL_FLASH_HZ`, `PULSE_FREQUENCY_HZ`) and brightness are tunable on Dashboard under `Lebot2_LEDStatus`.

---

## Autonomous Mode

Autonomous runs automatically after pressing Start (if game state is set to Autonomous during init). The driver does not need to do anything during autonomous, but can take over with the joystick if needed — any significant input aborts the current mission.

### Strategy

The flow depends on which start you selected in init (A = audience, Y = goal):

**Goal wall start:**
1. Back up to the fire position (spinning up on the way)
2. Launch preloaded balls
3. Collect ball row 1 → return to fire → launch
4. Collect ball row 2 → return to fire → launch
5. Collect ball row 3 → return to fire → launch

**Audience wall start:** already at the fire spot — spin up, wait briefly for the turret to lock (up to `CENTERING_TIMEOUT_SECONDS`), launch preloads, then collect rows in reverse order (3 → 2 → 1) with returns to fire between.

### Autonomous Coordination Options (partner play)

Set these to give way to a partner team or grab LEAVE points. Several behave **differently for goal vs audience starts.**

- **Selective abort** — `ABORT_AFTER_ROWS` (init: **D-pad Right**, or Dashboard `Lebot2_Autonomous`). How many rows to collect before parking out of the way:
  - `ALL` (default) = collect every row
  - `0` = launch preloads only, collect no rows
  - `1` / `2` = collect that many rows, then park
  - **Park destination by side:** goal start parks at `FIRE_2` — **note this is a firing position INSIDE the launch triangle, so it does NOT earn LEAVE points** and doesn't really clear your alliance area. Audience start drives to the **opposing alliance's BASE** (outside — clears the lane and gets LEAVE). Both are Dashboard-tunable (`ALT_POSITION_GOAL`, `ALT_POSITION_AUDIENCE`).
- **`SKIP_LAUNCH`** (Dashboard) — skip launching entirely and just drive off the line for **LEAVE** points (goes to opposing base — the one option that reliably leaves the zone).
- **`CENTERING_TIMEOUT_SECONDS`** (Dashboard) — **audience only**: how long to wait for turret lock before the first launch.

> **Only `SKIP_LAUNCH` (or an audience-start abort) actually leaves the launch zone for LEAVE points.** There is currently no goal-side "fire preloads then leave" — the goal-side abort and `leave` both park inside the triangle.

### ⚠️ Currently broken / useless — do NOT tune these

These appear on Dashboard but **do nothing useful right now** (their code paths are disabled or misbehave). Tuning them live will mislead you:

- **`leave`** — **broken/useless.** Goal-side only; supposed to be a "leave" play but it just fires preloads, returns to a **firing position** (inside the triangle), and stops — **no LEAVE points, doesn't leave.** Leftover one-off; ignore it until rebuilt.
- `DO_OPEN_SESAME` — gate release is fully commented out; no Open Sesame happens.
- `MIN_TIME_FOR_ROW` — the time-based row-skip is disabled; rows are not skipped on low time.
- `START_AT_GOAL_WALL` — overridden by the gamepad start-position selection (A/Y); has no effect.
- `AUTON_LAUNCH_SPACER_TIME`, `FIRST_ANGLE_OFFSET`, `SECOND_ANGLE_OFFSET` — only used by the retired `execute()` path, not the live auton.

---

## Test Mode

Test mode is for tuning and diagnostics. It enables debug telemetry and replaces the normal button mappings with tuning missions.

**Drive still works normally** — left stick forward/back, right stick turn.

### Tuning Missions

Each button starts an automated test. Only one can run at a time. Results are logged to CSV and shown in telemetry.

| Input | Action |
|---|---|
| **A** | **Rotation Test** — Four 90-degree turns. Measures heading accuracy and position drift. |
| **B** | **Square Test** (position-based) — Drives a square using position targets. Measures return-to-start error. |
| **Guide + B** | **Square Test** (RR trajectory) — Same square using Road Runner trajectories. |
| **X** | **Straight Line Test** (position-based) — Drives forward and back. Measures distance accuracy. |
| **Guide + X** | **Straight Line Test** (RR trajectory) — Same test using Road Runner trajectories. |
| **Y** | **Turn Accuracy Test** — Sequence of 45-degree, 90-degree, and 180-degree turns. Measures heading precision at each. |
| **Right Bumper** | **Ramsete Test** — Drives a trajectory with heading disturbance. Tests trajectory tracking recovery. |
| **D-pad Up** | **Drift Test** — Repeated turn cycles measuring cumulative position drift to detect odometry errors. |
| **Left Bumper** | **Health Check** — Pre-match system verification (see below). |
| **Back** | **Abort** current running mission. |

**Guide (Home)** is used as a shift key — hold it while pressing B or X to get the RR trajectory variant instead of position-based.

---

## Pre-Match Health Check

The health check is a sequenced test that verifies every major system on the robot. Run it before each match to catch disconnected motors, dead sensors, or calibration issues.

### When to Run

- Before every match, after the robot is powered on and initialized
- After any hardware change (swapping motors, reconnecting cables)
- If the robot behaved unexpectedly in a previous match

### Requirements

- Game state must be set to **Test** (use right trigger during init to cycle)
- Robot needs ~3 feet of clear space for turns (no balls needed)
- An AprilTag in view is helpful for the vision check but not required

### How to Run

1. Set game state to **Test** during init
2. Press **Start** to begin the OpMode
3. Press **Left Bumper** to start the health check
4. **Watch the robot** — it will turn in place, then spin up the flywheel
5. **Watch for the star direction prompt** — telemetry will show:
   ```
   >>> CONFIRM: Did star spin correctly? A=YES  B=NO
   ```
   Press **A** if the star spun the correct direction, **B** if not. If you don't respond within 10 seconds, it times out as FAIL.
6. The robot will then briefly run the intake and conveyor
7. Results appear in telemetry as PASS/FAIL for each check

Press **Back** at any time to abort the health check.

### What It Checks

| Check | What It Tests | Pass Criteria |
|---|---|---|
| **Battery** | Control hub voltage | >= 12.8V |
| **Pinpoint** | Odometry computer status | Device reports READY, encoders responding |
| **Turn Heading** | Gyro / odometry accuracy | Heading error < 5 degrees after four 90-degree turns |
| **Turn Position** | Odometry drift during turns | Position drift < 3 inches after four in-place turns |
| **Flywheel Speed** | Both flywheel motors reach target | Reaches 750 deg/s within timeout |
| **Flywheel Amps** | Both motors drawing current | Both > 0.05A, difference < 0.5A |
| **Flywheel Encoders** | Both motors spinning at similar rates | Slower/faster ratio > 0.7 |
| **Launch / Conveyor** | Fire sequence runs conveyor | Conveyor draws current during fire. Skipped if flywheel didn't reach speed. |
| **Star Direction** | Star trigger spins correct way | Driver visual confirmation (A=yes, B=no) |
| **Intake Motor** | Intake belt draws current | Current > 0.1A during 1-second run |
| **Conveyor (intake)** | Conveyor draws current during intake | Current > 0.1A during 1-second run |
| **Vision** | Limelight sees targets | Informational only (always passes) — reports whether target/botpose detected |

### Reading the Results

After completion, telemetry shows each check with PASS or FAIL and supporting data:

```
Battery:            PASS -- 13.2V (min 12.8)
Pinpoint:           PASS -- status=READY parTicks=4521
Turn Heading:       PASS -- 2.3 deg err (max 5.0)
Turn Position:      PASS -- 1.1" drift (max 3.0)
Flywheel Speed:     PASS -- reached 750 deg/s
Flywheel Amps:      PASS -- main=0.45A helper=0.42A diff=0.03
Flywheel Encoders:  PASS -- ratio=0.98 (main=3201 helper=3145)
Conveyor (launch):  PASS -- peak=0.32A (min 0.10)
Star Direction:     PASS -- driver confirmed OK
Intake Motor:       PASS -- peak=0.65A (min 0.10)
Conveyor (intake):  PASS -- peak=0.28A (min 0.10)
Vision:             PASS -- target=YES botpose=YES
RESULT:             ALL CHECKS PASSED
```

### Troubleshooting Failures

| Failure | Likely Cause | Fix |
|---|---|---|
| Battery FAIL | Battery low or dying | Swap battery |
| Pinpoint FAIL | I2C disconnected or not calibrated | Check Pinpoint cable, power cycle |
| Turn Heading FAIL | Odometry pods slipping or offset wrong | Check pod contact with ground, verify offsets in PinpointConfiguration doc |
| Turn Position FAIL | Odometry phantom translation | Check Y pod encoder direction in PinpointLocalizer |
| Flywheel Speed FAIL | Motor disconnected, PID not tuned, low battery | Check motor cables. Verify PIDF coefficients match Dashboard values. |
| Flywheel Amps FAIL (one near zero) | One motor physically disconnected but encoder connected | Reconnect motor power cable |
| Flywheel Encoders FAIL | One motor not spinning or encoder disconnected | Check both motor and encoder cables |
| Launch SKIPPED | Flywheel never reached speed | Fix flywheel issue first |
| Star Direction FAIL | Star spinning wrong way or not spinning | Check star servo direction/wiring |
| Intake Motor FAIL | Intake belt motor disconnected | Check intake motor cable |
| Conveyor FAIL | Conveyor motor disconnected | Check conveyor motor cable |

### Dashboard Thresholds

All pass/fail thresholds are tunable via FTC Dashboard under `Lebot2_Missions`:

| Parameter | Default | Description |
|---|---|---|
| `HEALTH_BATTERY_MIN` | 12.8V | Minimum acceptable voltage |
| `HEALTH_HEADING_TOLERANCE` | 5.0 deg | Max heading error after 4 turns |
| `HEALTH_POSITION_TOLERANCE` | 3.0 in | Max position drift after 4 turns |
| `HEALTH_FLYWHEEL_SPEED` | 750 deg/s | Target speed for flywheel check |
| `HEALTH_FLYWHEEL_AMP_DIFF` | 0.5A | Max current difference between motors |
| `HEALTH_FLYWHEEL_ENCODER_RATIO` | 0.7 | Min ratio of slower to faster motor |
| `HEALTH_INTAKE_MIN_AMPS` | 0.1A | Min current to confirm motor running |
| `HEALTH_CHECK_TIMEOUT` | 5.0s | Timeout per phase |
