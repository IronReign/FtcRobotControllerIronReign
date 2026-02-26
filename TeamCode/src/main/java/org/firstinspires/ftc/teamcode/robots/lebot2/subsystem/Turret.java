package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.lebot2.FieldMap;
import org.firstinspires.ftc.teamcode.robots.lebot2.Robot;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDrivePinpoint;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "Lebot2_Turret")
public class Turret implements Subsystem {

    public static double BUFFER = 2.5;

    private ElapsedTime lostVisionTimer = new ElapsedTime();
    public static double LOST_VISION_TIME = .2;

    private ElapsedTime lockedVisionTimer = new ElapsedTime();
    public static double LOCKED_VISION_TIME = .2;

    public static double LAUNCH_TOLERANCE = 1;
    public boolean readyToLaunch = false;

    // ==================== ENCODER / MECHANICAL LIMITS ====================
    // Calibrate these on the physical robot:
    //   Center the turret manually, read encoder -> TURRET_CENTER_TICKS
    //   Rotate CW to stop, read encoder -> CW_LIMIT_TICKS
    //   Rotate CCW to stop, read encoder -> CCW_LIMIT_TICKS
    //   TICKS_PER_DEGREE = (CW_LIMIT_TICKS - CCW_LIMIT_TICKS) / 210.0
    public static int TURRET_CENTER_TICKS = 0;
    public static int FULL_ROTATION_TICKS = 2820;
    public static int CW_LIMIT_TICKS = -823;     // placeholder - calibrate on robot
    public static int CCW_LIMIT_TICKS = 823;    // placeholder - calibrate on robot
    public static double TICKS_PER_DEGREE = FULL_ROTATION_TICKS / 360.0;

    // Derived degree limits (computed from ticks, but also dashboard-tunable for quick adjustment)
    public static double CW_LIMIT_DEG = -105;
    public static double CCW_LIMIT_DEG = 105;

    // ==================== PID PARAMS ====================
    public static PIDCoefficients TURRET_PID = new PIDCoefficients(0.04, 0.00005, 0.0075);
    public static double MAX_SPEED = 1.0;
    public static double TOLERANCE = 1.0;          // degrees - PID convergence threshold
    public static double INTEGRAL_CUTIN = 3.0;     // degrees
    public static double EMA_ALPHA = 0.5;
    public static double VISION_OFFSET = 0;        // camera center offset in degrees

    // ==================== HARDWARE ====================
    public static boolean REVERSE_MOTOR = true;
    private final DcMotorEx turretMotor;

    // ==================== DEPENDENCIES ====================
    private Vision vision;
    private TankDrivePinpoint driveTrain;

    // ==================== PID ====================
    private final PIDController turretPID;

    // ==================== STATE ====================
    public enum Behavior {
        TRACKING,   // Autonomous target seeking: vision when available, pose-based bearing fallback
        LOCKED      // PID holds turret at 0° (forward). Safety fallback.
    }

    public enum TargetingPhase {
        VISION_TRACKING,    // Vision has target, PID on tx
        POSE_SEEKING,       // No vision, PID on pose-computed bearing
        AT_LIMIT,           // Desired angle clamped to mechanical limit
        LOCKED_FORWARD      // Behavior.LOCKED - holding at 0°
    }

    private Behavior behavior = Behavior.LOCKED;
    private TargetingPhase phase = TargetingPhase.LOCKED_FORWARD;

    // Sensor reads (Phase 1: readSensors)
    private int encoderTicks = 0;
    private double turretAngleDeg = 0;

    // Staged output (Phase 2: calc -> Phase 3: act)
    private double stagedPower = 0;

    // ==================== CONSTRUCTOR ====================

    public Turret(HardwareMap hardwareMap) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        if (REVERSE_MOTOR) {
            turretMotor.setDirection(DcMotorEx.Direction.REVERSE);
        }
        resetTurret();
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretPID = new PIDController(TURRET_PID);
        turretPID.setInputRange(-180, 180);
        turretPID.setOutputRange(-MAX_SPEED, MAX_SPEED);
        turretPID.setIntegralCutIn(INTEGRAL_CUTIN);
        turretPID.setContinuous(false);
        turretPID.setTolerance(TOLERANCE / 360 * 100);
        turretPID.setEmaAlpha(EMA_ALPHA);
        turretPID.enable();
    }


    public void resetTurret(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ==================== THREE-PHASE SUBSYSTEM ====================

    @Override
    public void readSensors() {
        encoderTicks = turretMotor.getCurrentPosition();
        if (TICKS_PER_DEGREE != 0) {
            turretAngleDeg = (encoderTicks - TURRET_CENTER_TICKS) / TICKS_PER_DEGREE;
        }
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // Update PID tuning from dashboard
        turretPID.setPID(TURRET_PID);
        turretPID.setOutputRange(-MAX_SPEED, MAX_SPEED);

        if(Math.abs(vision.getTx()) < VISION_OFFSET+LAUNCH_TOLERANCE){
            if(lockedVisionTimer.seconds() > LOCKED_VISION_TIME){
                readyToLaunch = true;
            }else{
                readyToLaunch = false;
            }

        }else{
            readyToLaunch = false;
            lockedVisionTimer.reset();
        }


        if (behavior == Behavior.LOCKED) {
            // Hold turret at 0° (forward)
            phase = TargetingPhase.LOCKED_FORWARD;
            double targetAngle = clampToLimits(0);
            turretPID.setSetpoint(targetAngle);
            turretPID.setInput(turretAngleDeg);
            stagedPower = turretPID.performPID();
            lostVisionTimer.reset();

        } else { // TRACKING
            if((lostVisionTimer.seconds() > LOST_VISION_TIME && (vision != null && !vision.hasTarget())) || vision == null){
//                if (driveTrain != null) {
//                    // Pose-based bearing fallback
//                    Pose2d currentPose = driveTrain.getPose();
//                    Pose2d goalPose = FieldMap.getPose(FieldMap.GOAL, Robot.isRedAlliance);
//                    double bearingRad = FieldMap.bearingTo(currentPose, goalPose);
//                    double chassisRad = currentPose.heading.toDouble();
//                    double desiredTurretDeg = normalizeDeg(Math.toDegrees(bearingRad - chassisRad));
//
//                    // Clamp target to mechanical limits
//                    double clampedTarget = clampToLimits(desiredTurretDeg);
//                    phase = (clampedTarget != desiredTurretDeg) ? TargetingPhase.AT_LIMIT : TargetingPhase.POSE_SEEKING;
//
//                    turretPID.setSetpoint(clampedTarget);
//                    turretPID.setInput(turretAngleDeg);
//                    stagedPower = turretPID.performPID();
//
//                }

            }else if (vision != null && vision.hasTarget()) {
                lostVisionTimer.reset();
                //within turning limits
                if(turretAngleDeg < CCW_LIMIT_DEG+BUFFER && turretAngleDeg > CW_LIMIT_DEG-BUFFER){
                    // Vision mode: error is tx directly (degrees off-center in camera frame)
                    phase = TargetingPhase.VISION_TRACKING;
                    turretPID.setSetpoint(VISION_OFFSET);
                    turretPID.setInput(vision.getTx());
                    stagedPower = turretPID.performPID();

                }else{
                    stagedPower = 0;
//                    if(turretAngleDeg < 0){
//                        turretPID.setSetpoint(CW_LIMIT_DEG);
//                        turretPID.setInput(turretAngleDeg);
//                        stagedPower = turretPID.performPID();
//                    }else{
//                        turretPID.setSetpoint(CCW_LIMIT_DEG);
//                        turretPID.setInput(turretAngleDeg);
//                        stagedPower = turretPID.performPID();
//                    }
//                    // Pose-based bearing fallback
//                    Pose2d currentPose = driveTrain.getPose();
//                    Pose2d goalPose = FieldMap.getPose(FieldMap.GOAL, Robot.isRedAlliance);
//                    double bearingRad = FieldMap.bearingTo(currentPose, goalPose);
//                    double chassisRad = currentPose.heading.toDouble();
//                    double desiredTurretDeg = normalizeDeg(Math.toDegrees(bearingRad - chassisRad));
//
//                    // Clamp target to mechanical limits
//                    double clampedTarget = clampToLimits(desiredTurretDeg);
//                    phase = (clampedTarget != desiredTurretDeg) ? TargetingPhase.AT_LIMIT : TargetingPhase.POSE_SEEKING;
//
//                    turretPID.setSetpoint(clampedTarget);
//                    turretPID.setInput(turretAngleDeg);
//                    stagedPower = turretPID.performPID();
                }
            } else{
                stagedPower = 0;
                lostVisionTimer.reset();
            }
//            else if (driveTrain != null) {
//                // Pose-based bearing fallback
//                Pose2d currentPose = driveTrain.getPose();
//                Pose2d goalPose = FieldMap.getPose(FieldMap.GOAL, Robot.isRedAlliance);
//                double bearingRad = FieldMap.bearingTo(currentPose, goalPose);
//                double chassisRad = currentPose.heading.toDouble();
//                double desiredTurretDeg = normalizeDeg(Math.toDegrees(bearingRad - chassisRad));
//
//                // Clamp target to mechanical limits
//                double clampedTarget = clampToLimits(desiredTurretDeg);
//                phase = (clampedTarget != desiredTurretDeg) ? TargetingPhase.AT_LIMIT : TargetingPhase.POSE_SEEKING;
//
//                turretPID.setSetpoint(clampedTarget);
//                turretPID.setInput(turretAngleDeg);
//                stagedPower = turretPID.performPID();
//
//            } else {
//                // No vision, no drivetrain - hold position
//                stagedPower = 0;
//            }
        }
    }

    @Override
    public void act() {
        turretMotor.setPower(stagedPower);
    }

    // ==================== PUBLIC API ====================

    public void setVision(Vision vision) {
        this.vision = vision;
    }

    public void setDriveTrain(TankDrivePinpoint driveTrain) {
        this.driveTrain = driveTrain;
    }

    public void setTracking() {
        behavior = Behavior.TRACKING;
    }

    public void setLocked() {
        behavior = Behavior.LOCKED;
    }

    public Behavior getBehavior() {
        return behavior;
    }

    public TargetingPhase getPhase() {
        return phase;
    }

    public boolean isLockedOnTarget() {
        return phase == TargetingPhase.VISION_TRACKING && turretPID.lockedOnTarget();
    }

    public double getTurretAngleDeg() {
        return turretAngleDeg;
    }

    public boolean isReadyToLaunch(){return readyToLaunch;}

    // ==================== HELPERS ====================

    private double clampToLimits(double angleDeg) {
        return Math.max(CCW_LIMIT_DEG, Math.min(CW_LIMIT_DEG, angleDeg));
    }

    private static double normalizeDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }

    // ==================== LIFECYCLE ====================

    @Override
    public void stop() {
        turretMotor.setPower(0);
        behavior = Behavior.LOCKED;
        stagedPower = 0;
    }

    @Override
    public void resetStates() {
        behavior = Behavior.LOCKED;
        phase = TargetingPhase.LOCKED_FORWARD;
        stagedPower = 0;
        turretPID.enable();
    }

    // ==================== TELEMETRY ====================

    @Override
    public String getTelemetryName() {
        return "Turret";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("Behavior", behavior);
        telemetry.put("Phase", phase);
        telemetry.put("Angle (deg)", String.format("%.1f", turretAngleDeg));
        telemetry.put("Power", String.format("%.3f", stagedPower));
        telemetry.put("Locked On", isLockedOnTarget());
        telemetry.put("Encoder", encoderTicks);
        telemetry.put("Ready to Launch? ", readyToLaunch);
        if (vision != null && vision.hasTarget()) {
            telemetry.put("tx", String.format("%.1f", vision.getTx()));
        }
        if (debug) {
            telemetry.put("Encoder", encoderTicks);
        }
        return telemetry;
    }
}
