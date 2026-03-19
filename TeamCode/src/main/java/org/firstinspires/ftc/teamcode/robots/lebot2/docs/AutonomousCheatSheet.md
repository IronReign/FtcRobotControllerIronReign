# Autonomous Configuration Cheat Sheet
## execute2() — Goal Wall Start (splines enabled)

Generated from direct code read — 2026-03-19

---

## 1. Strategy Layer (`Lebot2_Autonomous`)

| Parameter | Default | What it does |
|---|---|---|
| `START_AT_GOAL_WALL` | `true` | Selects goal vs audience init — but actually overridden by `robot.getStartingPosition()` in `init()` |
| `SKIP_LAUNCH` | `false` | If true, skips all launching — just drives to ALT_POSITION_AUDIENCE for LEAVE points |
| `SKIP_INITIAL_BACKUP` | `false` | Skip backup-to-fire if already at fire position (set `true` for audience start) |
| `FIRE_POSITION` | `1` (goal) / `4` (aud) | Which fire position to use; **auto-increments** after each launch cycle (`min(pos+1, MAX_FIRE_POSITION)`) |
| `MAX_FIRE_POSITION` | `3` (goal) / `6` (aud) | Ceiling for FIRE_POSITION increment |
| `ROW_START` | `0` (goal) / `2` (aud) | First ball row index |
| `ROW_END` | `2` (goal) / `0` (aud) | Last ball row index |
| `ROW_DIRECTION` | `+1` (goal) / `-1` (aud) | Direction through rows |
| `ABORT_AFTER_ROWS` | `-1` | Selective abort: -1=all rows, 0=no rows, N=after N rows then navigate to alt position |
| `ALT_POSITION_GOAL` | `"FIRE_2"` | Abort destination for goal start |
| `ALT_POSITION_AUDIENCE` | `"BASE"` | Abort destination for audience start (opposing alliance base) |
| `LOGGING_ENABLED` | `true` | CSV logging |

### execute2() Flow
```
INIT -> BACKUP_TO_FIRE -> LAUNCH -> BALL_ROW(0) -> RETURN_TO_FIRE -> LAUNCH -> BALL_ROW(1) -> ... -> COMPLETE
                                    ^ FIRE_POSITION increments each cycle
```
**Key difference from execute()**: No START_TARGETING / WAITING_TARGET states. Goes straight BACKUP -> LAUNCH. No vision centering step before launch.

### Dead / Dormant in execute2()
- `DO_OPEN_SESAME` — gate release code fully commented out
- `MIN_TIME_FOR_ROW` — time-based skip logic fully commented out
- `GATE_BEFORE_ROW` — set but never checked
- `CENTERING_TIMEOUT_SECONDS` — used only in execute()
- `FIRST_ANGLE_OFFSET`, `SECOND_ANGLE_OFFSET` — used only in execute()
- `AUTON_LAUNCH_SPACER_TIME` — assignment to launcher commented out
- `oldRobot` — never read anywhere meaningful

---

## 2. Field Waypoints (`Lebot2_FieldMap`)

All waypoints defined for RED alliance. Blue is auto-reflected across X axis (Y negates, heading negates).

### Firing Positions

| Name | Base (x, y, heading) | Angle Offset Parameter | Effective heading |
|---|---|---|---|
| `FIRE_1` | (-24.9, 28.0, 135) | `FIRE_1_ANGLE_OFFSET` = -6 | 129 |
| `FIRE_2` | (-22, 22, 90) | `FIRE_2_ANGLE_OFFSET` = -9 | 81 |
| `FIRE_3` | (-16.6, 17.1, 90) | none | 90 |
| `FIRE_4` | (56.7, 20.8, 139) | `FIRE_4_ANGLE_OFFSET` = 0 | 139 |
| `FIRE_5` | (58.7, 21.5, 88) | none | 88 |
| `FIRE_6` | (58.7, 21.5, 88) | none | 88 (same as 5 — placeholder) |

### Starting Positions

| Name | (x, y, heading) |
|---|---|
| `START_GOAL` | (-50.46, 51.24, 135) |
| `START_AUDIENCE` | (58.7, 20.8, 139) |

### Ball Rows (Red Alliance base coords)

| Row | START base | END base |
|---|---|---|
| 1 | (-14.17, 24.48, 90) | (-14.17, 49.35, 90) |
| 2 | (10.24, 25.98, 90) | (10.24, 46.35, 90) |
| 3 | (34.65, 25.98, 90) | (34.65, 51.35, 90) |

### Row Offset Parameters

| Parameter | Default | Effect |
|---|---|---|
| `ROW_Y_START_OFFSET` | `4` | Added to _START Y. **Positive pushes start further from wall** (closer to end) |
| `ROW_X_OFFSET` | `0` | Subtracted from all row X coords |
| `BALL_ROW_BLUE_X_OFFSET` | `0` | Additional X offset for blue alliance ball rows (intake asymmetry) |
| `OFFSET` | `2` | Used in FIRE_4 X calculation only: `58.7 - OFFSET` |

### Spline X Offsets (applied only when `USE_SPLINES=true`, to _START only)

These shift the row approach point laterally so the spline entry arc lines up cleanly.

**Goal start** (approaching from negative X / FIRE_1 side):

| Parameter | Default | Applied to |
|---|---|---|
| `GOAL_ROW_1_SPLINE_X_OFFSET` | `-4` | Row 1 start |
| `GOAL_ROW_2_SPLINE_X_OFFSET` | `-7` | Row 2 start |
| `GOAL_ROW_3_SPLINE_X_OFFSET` | `-4` | Row 3 start |

**Audience start** (approaching from positive X / FIRE_4 side):

| Parameter | Default | Applied to |
|---|---|---|
| `AUD_ROW_1_SPLINE_X_OFFSET` | `6` | Row 1 start |
| `AUD_ROW_2_SPLINE_X_OFFSET` | `4` | Row 2 start |
| `AUD_ROW_3_SPLINE_X_OFFSET` | `0` | Row 3 start |

### Worked Example: Effective Row 1 START for Goal/Red

Base: (-14.17, 24.48)
+ ROW_Y_START_OFFSET(4) on Y
- ROW_X_OFFSET(0) on X
+ GOAL_ROW_1_SPLINE_X_OFFSET(-4) on X
= **(-18.17, 28.48, 90)**

### Flywheel Default Speeds

| Parameter | Default | When used |
|---|---|---|
| `FIRE_1_DEFAULT_DPS` | `1050` | Fallback when vision can't solve distance |
| `FIRE_4_DEFAULT_DPS` | `1050` | Same for audience |

---

## 3. Motion System (`Lebot2_TankDriveActions`)

| Parameter | Default | What it does |
|---|---|---|
| `USE_SPLINES` | `true` | Spline trajectories for ball row approach; turn-drive-turn for everything else |
| `INTAKE_VEL_INCHES_SEC` | `10.0` | Velocity through ball row (lineToY segment of spline trajectory) |
| `INITIAL_TURN_SKIP_TOLERANCE` | `5.0` | Skip initial turn if heading within this many degrees of bearing |
| `FINAL_TURN_SKIP_TOLERANCE` | `10.0` | Skip final turn if heading within this (turret handles fine aim) |
| `SPLINE_PRETURN_THRESHOLD_DEG` | `30.0` | If heading differs from row bearing by more than this, add pre-turn before spline |
| `POSITION_TOLERANCE` | `1.5"` | Along-track completion threshold for PositionDriveAction |
| `MAX_DRIVE_POWER` | `1.0` | Cap on PositionDriveAction motor output |
| `SETTLING_TIMEOUT_MS` | `500` | Safety timeout after first overshoot crossing |

### How Spline Row Pickup Works (buildRowTrajectory)
1. Compute bearing from current pose to row start
2. If heading difference > `SPLINE_PRETURN_THRESHOLD_DEG`: **turn -> intake -> spline -> lineToY**
3. If heading difference <= threshold: **intake -> spline -> lineToY**
4. Spline approach uses default velocity; lineToY segment uses `INTAKE_VEL_INCHES_SEC`

### How Return-to-Fire Works (turn-drive-turn)
1. `shouldDriveReversed()` picks forward or reverse based on least turning
2. `LazyBearingTurnAction` — turn to face (or face away from) target; skipped if within `INITIAL_TURN_SKIP_TOLERANCE`
3. `PositionDriveAction` — PID drive to target position
4. `LazyTurnAction` — correct final heading; skipped if within `FINAL_TURN_SKIP_TOLERANCE`

### Position Drive PID

| Parameter | Default |
|---|---|
| `DISTANCE_PID` | P=0.04, I=0.04, D=2.0 |
| `DIST_PID_I_CUTIN` | 8.0" (I term only active within this distance) |
| `HEADING_DRIVE_PID` | P=0.03, I=0.03, D=0.001 |
| `HEAD_PID_I_CUTIN` | 5.0 (I term only active within this error) |

### Slew Rate Limiting (anti-wheelie)

| Parameter | Default | Notes |
|---|---|---|
| `ACCEL_SLEW_RATE` | 0.05/tick | Max power increase per loop (~50ms) |
| `DECEL_SLEW_RATE_FORWARD` | 0.08/tick | Unused — forward decel is unslewed (PID has full authority) |
| `DECEL_SLEW_RATE_REVERSE` | 0.04/tick | Gentler decel in reverse to prevent wheelies |

---

## 4. RoadRunner Turn/Trajectory Profile (`Lebot2_TankDrivePinpoint`)

### Trajectory Profile Params (inside PARAMS)

| Parameter | Default | What it does |
|---|---|---|
| `maxWheelVel` | `25` in/s | Velocity cap for spline/trajectory segments (NOT a hard ceiling — it's the profile planner's max) |
| `minProfileAccel` | `-30` in/s^2 | Max deceleration |
| `maxProfileAccel` | `50` in/s^2 | Max acceleration |
| `maxAngVel` | `PI` rad/s | Turn speed cap (shared path + turn) |
| `maxAngAccel` | `PI` rad/s^2 | Turn acceleration cap |
| `trackWidthInches` | `14.0` | Kinematics track width (live-tunable, rebuilt on change) |

### Turn Controller (RR motion-profiled turns used by LazyTurnAction)

| Parameter | Default |
|---|---|
| `turnGain` | 25.0 |
| `turnVelGain` | 3.0 |
| `turnIGain` | 1.0 |
| `turnICutIn` | 5.0 (only integrate when error < 5 degrees) |
| `turnFeedforwardScale` | 0.0 (pure feedback, no feedforward) |
| `turnCompleteTolerance` | 5.0 degrees |
| `turnCompleteVelTolerance` | 0.2 rad/s |
| `turnCompleteTimeout` | 2.0s (safety net if PID oscillates) |

### Vision Centering PID (used by execute() targeting, NOT by execute2())

| Parameter | Default |
|---|---|
| `VISION_PID` | P=0.022, I=0.01, D=0.013 |
| `VISION_PID_LONG` | P=0.03, I=0.01, D=0.013 |
| `VISION_OFFSET` | 0 |
| `VISION_TOLERANCE` | 1 degree |
| `VISION_INTEGRAL_CUTIN` | 3.0 degrees |
| `VISION_ALPHA` | 0.5 (EMA smoothing) |
| `CENTERING_MAX_SPEED` | 0.5 |

### Distance Tracking PID (for vision distance maintenance — tuning missions)

| Parameter | Default |
|---|---|
| `DISTANCE_PID` | P=0.02, I=0.0, D=0.01 |
| `DISTANCE_TOLERANCE` | 2.0" |
| `TARGET_DISTANCE_INCHES` | 30.0" |
| `DISTANCE_MAX_SPEED` | 0.7 |

---

## 5. Missions Layer (`Lebot2_Missions`)

| Parameter | Default | What it does |
|---|---|---|
| `NAVIGATION_TIMEOUT_SECONDS` | `10.0` | Timeout for navigate-to-fire and ball group navigation |
| `INTAKE_TIMEOUT_SECONDS` | `5.0` | Added to nav timeout for total ball group timeout (nav + intake = 15s max) |
| `INTAKE_BALL_ROW_TIME` | `0` | Dwell at end of row; auto-set to `INTAKE_BALL_ROW_4_TIME` for row index 3 |
| `INTAKE_BALL_ROW_4_TIME` | `2` | Dwell duration for human player row (seconds) |
| `LAUNCH_TIMEOUT_SECONDS` | `10` | Timeout for launch mission |
| `INTAKE_DRIVE_POWER` | `0.23` | Drive speed through row in turn-drive-turn mode (only when `USE_SPLINES=false`) |
| `DUMMY_LAUNCH_MODE` | `false` | Skip real launch, just wait `DUMMY_LAUNCH_DURATION` |
| `DUMMY_LAUNCH_DURATION` | `2.0` | Seconds to wait in dummy mode |

### Dead / Dormant in execute2()
- `PRESS_TIMEOUT_SECONDS` / `PRESS_DURATION_SECONDS` — open sesame never called
- `INTAKE_DRIVE_POWER` — only active when `USE_SPLINES=false`
- `DUMMY_LAUNCH_MODE` — presumably left from testing

---

## 6. Launcher (`Lebot2_Launcher`)

| Parameter | Default | Set by Autonomous |
|---|---|---|
| `STAR_FEEDING` | `1.0` | Goal init sets `1.0`, audience sets `0.7` |
| `SPEED_MULTIPLIER` | `0.9` | Multiplier on vision-computed flywheel speed |
| `FIRING_TIME` | `1.5s` | Hard timeout for firing sequence |
| `PULSED_FIRING` | `true` | Feed/pause cadence between balls |
| `LAUNCH_SPACER_TIMER` | `0.5s` | Time between star rotations |
| `LAUNCH_SPACER_TIMER_LAST` | `1.0s` | Longer spacer for last ball (no ball behind pushing) |
| `FEED_GOAL` | `-0.7` | Belt feed power for goal distance |
| `FEED_AUDIENCE` | `-0.5` | Belt feed power for audience distance |
| `VISION_OFFSET_GOAL` | `0` | Vision offset for goal-side shots |
| `VISION_OFFSET_AUDIENCE` | `-2` | Vision offset for audience-side shots |

---

## 7. Relocalizer (`Lebot2_Relocalizer`)

| Parameter | Default |
|---|---|
| `ENABLED` | `false` (off by default due to field position jumps) |
| `MIN_SAMPLES` | 3 |
| `MAX_SAMPLES` | 15 |
| `WINDOW_MS` | 500 |
| `MAX_SPREAD_INCHES` | 6.0 |
| `MAX_HEADING_SPREAD_DEG` | 5.0 |

`relocalizer.apply()` is called after each launch cycle in execute2(). When ENABLED=false, update() flushes every cycle so apply() is always a no-op.

---

## 8. Init-Time Configuration (set by Autonomous.init())

These values are **programmatically set** based on starting position. Dashboard overrides applied AFTER init() will take effect.

### Goal Wall Start
```
SKIP_LAUNCH = false
FIRE_POSITION = 1, MAX_FIRE_POSITION = 3
STAR_FEEDING = 1.0
DistanceHint = NEAR
ROW_START = 0, ROW_END = 2, ROW_DIRECTION = +1
GATE_BEFORE_ROW = 2
SKIP_INITIAL_BACKUP = false
BALL_ROW_BLUE_X_OFFSET = 0
```

### Audience Wall Start
```
SKIP_LAUNCH = false
FIRE_POSITION = 4, MAX_FIRE_POSITION = 6
STAR_FEEDING = 0.7
DistanceHint = FAR
ROW_START = 2, ROW_END = 0, ROW_DIRECTION = -1
GATE_BEFORE_ROW = 0
SKIP_INITIAL_BACKUP = true
BALL_ROW_BLUE_X_OFFSET = 0
```

---

## Quick Reference: What to Tune for Common Issues

| Problem | Parameters to adjust |
|---|---|
| Robot overshoots fire position | `POSITION_TOLERANCE`, `SETTLING_TIMEOUT_MS`, `DISTANCE_PID` |
| Turns too slow / oscillating | `turnGain`, `turnVelGain`, `turnIGain`, `turnCompleteTolerance` |
| Missing balls in row | `ROW_Y_START_OFFSET`, spline X offsets, `INTAKE_VEL_INCHES_SEC` |
| Row approach arc is wrong | `GOAL_ROW_*_SPLINE_X_OFFSET` (or AUD), `SPLINE_PRETURN_THRESHOLD_DEG` |
| Balls not launching far enough | `SPEED_MULTIPLIER`, `FIRE_*_DEFAULT_DPS`, `FIRE_*_ANGLE_OFFSET` |
| Firing too slow between balls | `LAUNCH_SPACER_TIMER`, `PULSED_FIRING`, pulsed timing params |
| Robot wheelies on decel | `DECEL_SLEW_RATE_REVERSE`, `ACCEL_SLEW_RATE`, `MAX_DRIVE_POWER` |
| Auton times out | `NAVIGATION_TIMEOUT_SECONDS`, `LAUNCH_TIMEOUT_SECONDS` |
| Skip rows to save time | `ABORT_AFTER_ROWS` (0=launch only, 1=one row, etc.) |
| Blue alliance offset issues | `BALL_ROW_BLUE_X_OFFSET` |
