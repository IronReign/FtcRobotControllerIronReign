# RI2D2 Catapult Tuning Guide

Bring-up and test procedure for the elastic catapult: servo endpoints, spool positions, capture dwell, and the failure modes worth knowing before you put a full draw on it.

**Work the steps in order.** Each one depends on the one above it — you can't find a spool position before the encoder is zeroed, and you shouldn't put elastic on anything until the dry run passes.

| | |
|---|---|
| OpMode | `TwoDays` (group `twoDays`) |
| Dashboard group | `newBot_Catapult` |
| Motor | `catapult` (DcMotorEx, encoder) |
| Servo | `catapultRelease` |
| Gamepad | Gamepad 1 |
| Source | `robots/ri2d2/subsystem/Catapult.java` |

> ⚠️ **Stored energy.** A drawn catapult is a loaded spring with a 4-ball bucket on the end. Keep hands, faces and the boom's swing path clear any time `Ticks` is above `SLACK_TICKS`, whether or not the code thinks it's latched. Do the whole first pass of this guide with the **elastic disconnected**.

---

## What's holding the boom

Exactly one of two things is responsible for a cocked boom at any moment. Almost every way this mechanism can hurt someone or jam itself is a bad handoff between them.

**The winch** — `RETRACTING`, `CAPTURING`, `HOLDING`
String under tension, motor powered under RUN_TO_POSITION. This is the only thing holding the boom until the latch is confirmed.

**The latch** — `READY`, and while driving around armed
Pulled down onto the boom pin by a weak rubber band. The servo's job is to hold it **up and out of the way**, never to push it down. It isn't secure when it first drops — only the weak band holds it there. Slackening lets the boom rise onto it, and that tension is what actually seats it.

**The handoff** — `CAPTURE_DWELL_MS`
Neither the servo nor the rubber band reports back. Time is the only confirmation there is, which makes the dwell the single most safety-relevant number in this file.

---

## State flow

One press of **Y** runs the whole arming chain and stops at READY.

```
RETRACTING -> CAPTURING -> SLACKENING -> READY -> LAUNCHING -> FIRED
```

- The latch is held **open** through all of `RETRACTING` and only let down at the `CAPTURING` transition. Dropping it before the pin is underneath is the jam case.
- Any trouble with a cocked, unlatched boom diverts to `HOLDING`, where the winch keeps holding position rather than letting the elastic snap the boom back up.
- `READY` is unpowered and is the normal state to drive around in. The rubber bands manage the loose cord.

---

## Controls

| Input | Action |
|---|---|
| **Y** | **Arm** — runs the full chain to READY. No-op when already armed. |
| **X** | **Launch** — slackens first if needed, then opens the latch. |
| **B** | **Abort** — cancels. Holds position if the boom is cocked and unlatched. |
| **D-pad Up** | Jog spool in. Boom travels down, ticks increase. |
| **D-pad Down** | Jog spool out. Ticks decrease. |
| **Back** | Zero encoder at current position. **Setup only — never on a cocked boom.** |
| Right Bumper | Toggle intake (not catapult) |

---

## Step 1 — Config names and a clear bench

Robot up on blocks with the boom's full swing path clear. **Elastic disconnected. No balls.**

- Confirm the Driver Station config has a motor named `catapult` and a servo named `catapultRelease`.
- Init the `TwoDays` OpMode. Catapult telemetry prints in debug mode, so you should see State, Ticks, Armed, Latched, String, Launch ok and Release.
- Open FTC Dashboard and find the `newBot_Catapult` config group.

> ✅ **Confirm:** State reads `IDLE` and telemetry is updating.

---

## Step 2 — Servo endpoints

Tuning: `RELEASE_CAPTURED`, `RELEASE_RELEASED`

Boom up, elastic still off. Both values are live-editable in Dashboard.

- **RELEASED** is the servo holding the latch **up and clear** of the pin's path. Sight down the pin travel and make sure nothing clips.
- **CAPTURED** is the servo out of the way so the rubber band pulls the latch fully down. The servo should not be fighting the band at this end.
- Defaults are `0.25` captured and `0.70` released. Adjust, don't assume.

> ✅ **Confirm:** latch moves cleanly between both endpoints, with no buzzing or binding at either end.

---

## Step 3 — Motor direction and encoder zero

Tuning: `MOTOR_REVERSED`

- Rest the boom **up** with the string just barely taut. Press **Back**. `Ticks` should read 0.
- Tap **D-pad Up**. `Ticks` must count **up** and the boom must travel **down**.
- If it goes the other way, flip `MOTOR_REVERSED` **and restart the OpMode** — it's read once in the constructor and will not take effect live.

> ✅ **Confirm:** jog up raises ticks and lowers the boom; jog down reverses both.

---

## Step 4 — Find the pin point

Tuning: `RETRACT_TICKS`, `RETRACT_OVERSHOOT_TICKS`

> ⚠️ **`RETRACT_TICKS` is not maximum travel.** It's the point where the pin has just cleared far enough down for the latch to drop behind it.

- Jog **D-pad Up** in small taps until the latch, if released, would fall cleanly behind the pin. Drop it by hand to check. Read `Ticks` — that's `RETRACT_TICKS`.
- Set `RETRACT_OVERSHOOT_TICKS` (default 40) so the winch pulls a bit past that point and holds there through the dwell. Overshooting is harmless; coming up short misses the pin and jams the mechanism.
- Jog to `RETRACT_TICKS + RETRACT_OVERSHOOT_TICKS` and verify the mechanism **does not bottom out** or bind at that position.

> ✅ **Confirm:** at the overshoot position the latch drops behind the pin by hand with obvious clearance, and nothing is hard against a stop.

---

## Step 5 — Find the slack position

Tuning: `SLACK_TICKS`

- From the retract position with the latch down on the pin, jog **D-pad Down** until the boom settles onto the latch and the string goes visibly loose.
- Keep going. You need enough loose cord that the boom can reach **full launch travel** without the string ever coming taut — a string that snubs mid-throw kills the shot or breaks.
- Read `Ticks` — that's `SLACK_TICKS`. It must be well below `RETRACT_TICKS`.
- Swing the boom up by hand through its full arc and watch the cord. If it tightens anywhere, lower `SLACK_TICKS`.

The launch gate is `Ticks <= SLACK_TICKS + POSITION_TOLERANCE`.

> ✅ **Confirm:** boom reaches full travel by hand with slack cord the whole way, and telemetry reads `Launch ok: YES`.

---

## Step 6 — Time the capture dwell

Tuning: `CAPTURE_DWELL_MS`

This covers the servo's travel **plus** the weak band pulling the latch all the way down. Neither reports back, so this timer is the only thing standing between the elastic and a half-seated latch.

- With no load, drive the servo from RELEASED to CAPTURED and time how long the latch takes to fully settle. Phone slow-mo is worth the two minutes here.
- Set `CAPTURE_DWELL_MS` to roughly **double** what you measured. Default is 500 ms.
- This costs a fraction of a second per cycle. **Do not trim it to save time.**

> ✅ **Confirm:** dwell is at least 2× the observed latch settle time.

---

## Step 7 — Dry run, no elastic

Press **Y** and watch the chain. This is the rehearsal that catches ordering bugs while nothing can hurt you.

- Latch stays **up** for all of `RETRACTING`. If it drops early, stop — that's your jam.
- Latch drops only as state changes to `CAPTURING`.
- Winch audibly keeps holding through `CAPTURING`, then pays out in `SLACKENING`.
- Ends at `READY` with `Armed: ARMED` and the motor quiet.
- Press **X** — Release flips to RELEASED, state runs `LAUNCHING` then `FIRED`.
- Press **Y** while armed — nothing should happen. That's deliberate.

> ✅ **Confirm:** three clean arm/fire cycles in a row with no manual help.

---

## Step 8 — Add elastic, lowest tension first

> ⚠️ **Do not start this step until Step 7 has passed three consecutive cycles.** From here the boom can move on its own.

- Reconnect the elastic at its **lowest** draw. Everyone clear of the arc.
- Run the cycle. Watch specifically for the boom twitching back up during the `SLACKENING` transition — that means the dwell is short.
- Step the draw up one notch at a time, re-running the full cycle at each notch.
- Recheck `SLACK_TICKS` at full draw: the boom throws further and faster than it did by hand.

> ✅ **Confirm:** clean cycles at full draw with no boom movement between `CAPTURING` and `READY`.

---

## Step 9 — Live fire with balls

- Load the bucket. It's a mix of both ball sizes, so check nothing fouls the latch or the cord at rest.
- Fire downrange only, with the field clear.
- Then drive it: arm, run a full teleop lap with hard accelerations and turns, and confirm the cord hasn't despooled or fouled before firing.

> ✅ **Confirm:** armed robot survives a driving lap and still fires clean.

---

## Telemetry reference

| Field | Meaning | Armed & idle |
|---|---|---|
| `State` | Where the sequence is | `READY` |
| `Ticks` | Live spool encoder position | ≈ `SLACK_TICKS` |
| `Armed` | Latched **and** slack — loaded and safe to drive | `ARMED` |
| `Latched` | Latch was released to the band and given the full dwell. **Not a sensor** — it's the timer's word. | `YES` |
| `String` | `TAUT` means the winch is still taking the elastic load | `slack` |
| `Launch ok` | The gate. The latch will not open unless this is `YES`. | `YES` |
| `Release` | Commanded servo end | `CAPTURED` |
| `Motor Power` | Debug. Should be 0 at READY — anything else means you're in `HOLDING`, not `READY`. | `0` |

---

## Tunable reference

All under `newBot_Catapult` in Dashboard.

| Constant | Default | What it does |
|---|---|---|
| `RETRACT_TICKS` | 1500 | Pin is far enough down for the latch to drop behind it |
| `RETRACT_OVERSHOOT_TICKS` | 40 | How far past that the winch pulls and holds during the dwell |
| `SLACK_TICKS` | 900 | String paid back out far enough to fire |
| `POSITION_TOLERANCE` | 25 | Slop allowed on the launch gate |
| `CAPTURE_DWELL_MS` | 500 | Winch holds the boom past the pin while the latch settles |
| `RETRACT_POWER` | 0.8 | Winding-down power |
| `SLACK_POWER` | 0.5 | Paying-out power |
| `JOG_POWER` | 0.3 | Manual D-pad jog |
| `RELEASE_CAPTURED` | 0.25 | Servo out of the way, band pulls latch down |
| `RELEASE_RELEASED` | 0.70 | Servo holding latch up, clear of the pin |
| `RETRACT_TIMEOUT_MS` | 4000 | Give up on retracting and go to `HOLDING` |
| `SLACK_TIMEOUT_MS` | 2000 | Give up on slackening |
| `LAUNCH_DWELL_MS` | 400 | Time for the boom to clear before calling it fired |
| `MOTOR_REVERSED` | false | **Constructor only — restart the OpMode** |

---

## When it misbehaves

| Symptom | Cause | Fix |
|---|---|---|
| **Latch misses the pin, mechanism jams** | Latch came down short of the pin | Raise `RETRACT_TICKS` or `RETRACT_OVERSHOOT_TICKS`. Re-do Step 4. |
| **Boom snaps back up during arming** | Winch slackened before the latch seated | Raise `CAPTURE_DWELL_MS`. **This is the dangerous one — fix before anything else.** |
| **Pressed X, ran `LAUNCHING` → `FIRED`, nothing launched** | String never reached slack. `SLACK_TIMEOUT_MS` expired, `READY` was entered anyway, and the latch gate then refused to open. Silent no-fire by design. | Check for `Launch ok: no`. Raise `SLACK_TIMEOUT_MS` or `SLACK_POWER`, or re-do Step 5. |
| **Ends in `HOLDING` after pressing Y** | Retract timed out before reaching the pin point. Latch is deliberately left open and the winch holds the boom. | Winch stalling, `RETRACT_POWER` too low, `RETRACT_TICKS` unreachable, or `RETRACT_TIMEOUT_MS` too short. |
| **Y does nothing** | Already armed. Arming starts by lifting the latch, which on a loaded pin **is** a launch. | Working as intended. Fire it or abort first. |
| **Ticks fall when jogging up** | Motor direction inverted | Flip `MOTOR_REVERSED` and **restart the OpMode**. |
| **Motor warm while sitting armed** | You're in `HOLDING`, not `READY`. `READY` is unpowered. | Read State. `HOLDING` means arming never completed. |

---

## Things that will cost you an hour

> ⚠️ **Restart required.** `MOTOR_REVERSED` is read once in the constructor. Changing it in Dashboard does nothing until you re-init the OpMode. Every other value in this file is live.

> ⚠️ **Never zero a cocked boom.** **Back** zeroes the encoder via `STOP_AND_RESET_ENCODER`, which drops motor power for an instant. On a drawn catapult that's a dropped boom. Setup only.

> ⚠️ **The servo initialises to CAPTURED on purpose.** If the robot gets powered down while armed, commanding RELEASED at init would lift the latch off a loaded pin and fire the catapult during init. Don't "tidy" this.

> **Telemetry flicker.** The Release readout compares against the live constant, so editing `RELEASE_RELEASED` in Dashboard while it's commanded makes the label read CAPTURED for a loop. Cosmetic — don't chase it.

> **Not a bug.** **B** on a cocked, unlatched boom keeps the motor powered and holds position instead of stopping. Aborting must never mean dropping the boom.

> **Prototype code.** ri2d2 is two-day throwaway. If the mechanism changes, change these numbers and move on — nothing here is meant to survive into the competition robot.
