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
| Guide (Home) | Set starting position: **Calibration** (field center, facing red goal) |
| Left Stick Y | Drive forward/backward (for pre-match positioning) |
| Right Stick X | Turn left/right |

**Starting positions** set the Pinpoint odometry's initial pose. Choose the one matching where the robot is physically placed:
- **Audience**: Touching audience wall, can see goal AprilTag at range
- **Goal**: Touching goal wall (default for competition)
- **Unknown**: Position unknown, relies on vision to localize
- **Calibration**: Field center facing red goal, for testing vision pose comparison

---

## Tele-Op Mode

### Driving

| Input | Action |
|---|---|
| Left Stick Y | Throttle (forward/backward) |
| Right Stick X | Turn (dampened by slow mode setting) |

Drive input is always active. If a mission or trajectory is running, significant joystick input (>10% deflection) automatically aborts it and returns control to the driver.

Normal turn dampening is 70%. Slow mode reduces it to 30%.

### Subsystem Controls

| Input | Action |
|---|---|
| **A** | **Toggle intake** (LOAD_ALL mode). Press once to start: overhead belt + conveyor run, auto-stop when loader sensors detect full. Press again to stop manually. |
| **B** | **Emergency stop all**. Stops intake, conveyor, launcher. Resets robot to manual control. |
| **X** | **Spin up flywheel**. Starts the launcher motors ramping to target speed. |
| **Y** | **Center on vision target**. Robot turns to face the detected AprilTag. Driver can override with joystick. |
| **Left Bumper** | **Toggle slow mode**. Reduces turn sensitivity for precise alignment. |
| **Right Bumper** | **Launch all**. Fires all loaded balls in sequence. Flywheel must be at speed (use X first). Intake assists at reduced power during the launch sequence. |

### Paddle / Trigger Controls

| Input | Action |
|---|---|
| D-pad Up | Paddle to RAMP position, enable pass-through mode |
| D-pad Down | Paddle to CUP position, disable pass-through mode |

### Intake Overrides

| Input | Action |
|---|---|
| D-pad Left | Simple intake on (continuous, no auto-stop) |
| D-pad Right | Eject balls (intake reverses, conveyor pushes forward) |

### Utility

| Input | Action |
|---|---|
| Start | Cycle Limelight tilt: Down / Straight / Up-Min / Up-Max |
| Back | Apply vision pose correction to Pinpoint (only when an AprilTag is visible). Gamepad rumbles to confirm. |
| Guide (Home) | Reset drive encoders and ball count |

### Launch Sequence Detail

The full launch cycle when you press **Right Bumper**:

1. Flywheel must already be spinning (press **X** first and wait for it to reach speed)
2. Right Bumper triggers LAUNCH_ALL behavior
3. Star trigger / ramp feeds balls into the spinning flywheel
4. Conveyor belt pushes balls toward the trigger
5. Intake runs at assist power for ~1 second to help feed
6. After all balls are fired, subsystems return to idle

If something goes wrong, press **B** to emergency stop everything.

---

## Autonomous Mode

Autonomous runs automatically after pressing Start (if game state is set to Autonomous during init). The driver does not need to do anything during autonomous, but can take over with the joystick if needed — any significant input aborts the current mission.

### Strategy (Goal Wall Start)

1. Back up from starting position to fire position
2. Target goal with vision, launch preloaded balls
3. Collect ball row 1, return to fire, target, launch
4. Collect ball row 2, return to fire, target, launch
5. Navigate to gate, release scored balls (Open Sesame)
6. Collect ball row 3 (includes released balls), return to fire, launch

The autonomous coordinator skips remaining rows if time is running low (< 5 seconds remaining per row cycle).

### Dashboard-Tunable Options

These can be changed on FTC Dashboard before the match:

- `START_AT_GOAL_WALL` — true for goal wall start, false for audience
- `DO_OPEN_SESAME` — whether to release the gate after rows 1 & 2
- `MIN_TIME_FOR_ROW` — minimum seconds needed to attempt another row cycle

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
