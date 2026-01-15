# Launcher TPU Ramp Design

This document explains the mechanical changes to the launcher system and the corresponding code updates.

## Mechanical Changes

### 1. High-Momentum Flywheel

**What changed:** Added rings with steel ball bearings at the flywheel rim.

**Why:** The added mass at the rim stores more angular momentum (I = mr^2), enabling rapid multi-ball firing with minimal speed loss between shots. The flywheel acts as an energy reservoir.

**Tradeoff:** Longer spin-up time due to increased moment of inertia, but this happens during navigation to the target.

### 2. TPU Ramp/Paddle

**What changed:** Replaced the rigid paddle with a flexible TPU (thermoplastic polyurethane) component that can assume three positions:

| Position | Servo Value | Description |
|----------|-------------|-------------|
| **CUP** | 0.30 | C-shape that traps balls during intake, preventing them from contacting the flywheel |
| **RAMP** | 0.4133 | Straightened ramp that creates a smooth path for the conveyor to push balls through |
| **LIFT** | 0.4133 | Raised position to push the last ball into flywheel contact (may need separate tuning) |

**Why:** The flexible TPU allows a single component to serve multiple functions:
- During intake: CUP position traps balls, preventing premature contact with flywheel
- During firing: RAMP position creates a smooth transition, conveyor pushes balls through continuously
- Edge case: LIFT position ensures the last ball reaches the flywheel

### 3. Removed Belt Reversal

**What changed:** The old design required reversing the conveyor belt briefly before each shot to relieve pressure. This is no longer needed.

**Why:** With the ramp design, balls flow smoothly through when the paddle is in RAMP position. The conveyor runs continuously during firing.

## Code Changes

### Launcher.java

#### Renamed Paddle Constants
```java
// Old names (unclear purpose)
PADDLE_DOWN, PADDLE_PASS, PADDLE_UP

// New names (describe mechanical function)
PADDLE_CUP   = 0.3     // C-shape trap during intake
PADDLE_RAMP  = 0.4133  // Straightened ramp for firing
PADDLE_LIFT  = 0.4133  // Raised to push last ball
```

#### Simplified State Machine
```
Old states: IDLE, REQUESTED, SPINNING_UP, READY, BELT_REVERSING2,
            FIRING, COOLDOWN, MANUAL, FIRST, NEXT, BUFFER, COMPLETE

New states: IDLE, SPINNING_UP, READY, FIRING, LIFTING, COMPLETE, MANUAL
```

State flow:
```
IDLE ─► SPINNING_UP ─► READY ─► FIRING ─► LIFTING ─► COMPLETE
  ▲                      │                              │
  │                      │ (fireRequested)              │
  │                      ▼                              │
  └──────────────────── MANUAL ◄────────────────────────┘
                                (if STAY_SPINNING_AFTER_FIRE)
                                      ▼
                                   READY
```

#### New Configuration Options
```java
// Timing
FIRING_TIME = 2.0  // seconds to allow all 3 balls through
LIFT_TIME = 0.3    // seconds to hold LIFT position

// Post-fire behavior (Dashboard tunable)
STAY_SPINNING_AFTER_FIRE = false  // true = keep flywheel spinning for faster follow-up
```

#### State Handler Summary

| State | Flywheel | Paddle | Conveyor | Transition |
|-------|----------|--------|----------|------------|
| IDLE | Off | CUP | - | setBehavior(SPINNING) → SPINNING_UP |
| SPINNING_UP | Target speed | CUP | - | isFlywheelAtSpeed() → READY |
| READY | Target speed | CUP | - | fire() → FIRING |
| FIRING | Target speed | RAMP | Running | Timeout → LIFTING |
| LIFTING | Target speed | LIFT | - | Timeout → COMPLETE |
| COMPLETE | Depends | CUP | - | → READY or IDLE |
| MANUAL | Target speed | External | - | For testing |

### Robot.java

#### Simplified LAUNCH_ALL Behavior

The LAUNCH_ALL behavior now delegates most logic to the Launcher:

1. Start flywheel spinning
2. Wait for READY state
3. Call fire() once
4. Wait for Launcher to complete the sequence
5. Return to MANUAL behavior

The Launcher handles FIRING → LIFTING → COMPLETE internally.

### DriverControls.java

Updated paddle control methods:
```java
// Old
robot.launcher.paddleUp();
robot.launcher.paddleDown();

// New
robot.launcher.paddleRamp();
robot.launcher.paddleCup();
```

## Dashboard-Tunable Parameters

The following can be adjusted in FTC Dashboard under `Lebot2_Launcher`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `PADDLE_CUP` | 0.3 | Servo position for CUP (ball trap) |
| `PADDLE_RAMP` | 0.4133 | Servo position for RAMP (firing) |
| `PADDLE_LIFT` | 0.4133 | Servo position for LIFT (last ball push) |
| `FIRING_TIME` | 2.0 | Seconds to hold FIRING state |
| `LIFT_TIME` | 0.3 | Seconds to hold LIFTING state |
| `MIN_LAUNCH_SPEED` | 1500 | Target flywheel speed (deg/sec) |
| `STAY_SPINNING_AFTER_FIRE` | false | Keep flywheel spinning after complete |

## Verification Checklist

1. [ ] **CUP position:** Balls are trapped, no flywheel contact during intake
2. [ ] **RAMP position:** Balls flow smoothly when conveyor runs
3. [ ] **LIFT position:** Last ball pushed into flywheel contact
4. [ ] **Spin-up time:** Measure time to reach 1500 deg/s with new flywheel mass
5. [ ] **3-ball launch:** All balls fired in sequence without manual intervention
6. [ ] **STAY_SPINNING toggle:** Works correctly from Dashboard

## Open Questions

1. **PADDLE_LIFT tuning:** Is 0.4133 correct, or does it need to be higher than PADDLE_RAMP?
2. **Ball detection:** Can we detect ball launches via flywheel speed derivative, or do we need an exit sensor?

## Related Files

- `Launcher.java` - Main launcher subsystem
- `Robot.java` - LAUNCH_ALL behavior coordination
- `DriverControls.java` - Driver paddle controls
- `Loader.java` - Conveyor belt and ball sensing
