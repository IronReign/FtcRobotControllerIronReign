package org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.utilMethods;

import java.util.HashMap;
import java.util.Map;

public class SwerveModule implements Subsystem {
    private String name;
    public DcMotorEx driveMotor;
    private CRServo yawServo;
    private DcMotorEx yawEncoder;
    private AnalogInput yawAnalog;
    private PIDController yawPID;
    private double ticksPerDegree;
    public boolean swerveAligned;

    private double targetAngle;       // in degrees, as a wrapped value (0-360)
    private double yawError;          // current yaw error (in degrees)
    private double thresholdAngle;    // if error is greater than this, do not enable drive
    private double desiredAnglePrivy;
    private boolean invertedDrive;    // whether the drive motor should be inverted

    // calibration constants - todo, change to private once done with dashboard tuning
    public static final double FAST_PWR      =  0.9;   // open-loop power while searching
    public static final double SLOW_PWR      =  0.25;  // open-loop power when refining
    public static final int    MIN_SCAN_TICS = 150;    // turn at least this long before giving up
    public static final double ANALOG_HYST   = 0.02;   // ignore noise around thresholds
    public static final double OFFSET_EPS    = 1.0;    // deg – when to accept “at offset”


    // ---------- CALIBRATION ---------------------------------------------------
    private enum CalibKind   { NONE, NORMAL, TICK_CHAR, OFFSET_CHAR }
    private enum CalibState  { IDLE, SEEK_FAST, SEEK_SLOW, FIND_EXTREMES,
        CENTER_FROM_POS, CENTER_FROM_NEG,
        TEMP_ZERO, GOTO_OFFSET, DONE,
        ONE_REV_START, ONE_REV_COUNT, BACKLASH_FWD,
        BACKLASH_REV, CAPTURE_END }               //  for other routines

    private CalibKind  calibKind  = CalibKind.NONE;
    private CalibState calibState = CalibState.IDLE;
    private int        scanTicks;               // generic counter
    private double     analogMin, analogMax;    // found extremes
    private double     midValue;                // boundary value
    private double     startAngle;              // for 360-deg count
    private int        revTicks;                // ticks in one revolution
    private double     backlashTicks;           // slop
    private final double alignmentOffsetDeg = 10;    // <- module-specific, ctor param


    /**
     * @param driveMotor      The drive motor (for wheel propulsion)
     * @param yawServo        The continuous rotation servo that steers the module
     * @param yawEncoder      The encoder providing feedback on the module’s yaw position
     * @param yawAnalog       The analog optical sensor for detecting the index position of the module's steering (yaw)
     * @param pidController   The PID controller instance to control the yaw
     * @param ticksPerDegree  Conversion factor from encoder ticks to degrees
     * @param thresholdAngle  Threshold (in degrees) below which the drive motor is enabled
     */
    public SwerveModule(String name, DcMotorEx driveMotor, CRServo yawServo, DcMotorEx yawEncoder, AnalogInput yawAnalog,
                        PIDController pidController, double ticksPerDegree, double thresholdAngle) {
        this.name = name;
        this.driveMotor = driveMotor;
        this.yawServo = yawServo;
        this.yawEncoder = yawEncoder;
        this.yawAnalog = yawAnalog;
        this.yawPID = pidController;
        this.ticksPerDegree = ticksPerDegree;
        this.thresholdAngle = thresholdAngle;
        this.invertedDrive = false;
    }

    /**
     * Call this periodically to update the yaw servo power.
     */
    @Override public void update(Canvas fieldOverlay) {
        double currentAngle = getCurrentAngle();
        // Compute the current error using a helper that returns the smallest difference (-180, 180)
        yawError = utilMethods.angleDifference(targetAngle, currentAngle);
        yawPID.setInput(currentAngle);
        yawPID.setSetpoint(targetAngle);
        double pidOutput = yawPID.performPID();
        // Negative because of the servo’s orientation (adjust sign as needed)
        yawServo.setPower(-pidOutput);
    }

    @Override public void stop() {
        driveMotor.setPower(0);
        yawServo.setPower(0);
    }

    @Override public void resetStates() {}

    @Override public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> t = new HashMap<>();
        if (debug) {
            t.put("steer current", getCurrentAngle());
            t.put("steer target", getTargetAngle());
            t.put("revolution ticks", getRevTicks());
            t.put("backlash", getBacklashTicks());
        }
        return t;
    }
    @Override public String getTelemetryName() { return "SwerveModule " + this.name; }
    /**
     * Sets the desired state for the module.
     * @param desiredAngle chassis-relative angle (deg)
     * @param speed        desired speed 0 – 1 (sign ignored; we invert internally)
     */
    public void setDesiredState(double desiredAngle, double speed) {
        // ---- 1. Harden inputs ----------------------------------------------
        if (Double.isNaN(desiredAngle))   desiredAngle = targetAngle; // hold last target
        if (Double.isNaN(speed))          speed        = 0;

        // store for telemetry
        desiredAnglePrivy = desiredAngle;

        double currentAngle = getCurrentAngle();
        double angleDiff    = utilMethods.angleDifference(desiredAngle, currentAngle);

        // ---- 2. Choose the shorter rotation path ---------------------------
        if (Math.abs(angleDiff) >= 90) {
            invertedDrive = true;
            targetAngle   = Utils.wrapAngle(desiredAngle - 180);
        } else {
            invertedDrive = false;
            targetAngle   = Utils.wrapAngle(desiredAngle);
        }

        // ---- 3. Check if yaw error is inside the steering threshold --------
        boolean withinPrimary   = Math.abs(angleDiff) < thresholdAngle;
        boolean withinOpposite  = Math.abs(angleDiff) > 180 - thresholdAngle &&
                Math.abs(angleDiff) < 180 + thresholdAngle;
        swerveAligned = withinPrimary || withinOpposite;

        // ---- 4. Apply drive only when aligned ------------------------------
        if (swerveAligned) {
            driveMotor.setPower(invertedDrive ? -speed : speed);
        } else {
            driveMotor.setPower(0);
        }

        // PID yaw servo always needs the target, even while drive is zero
        yawPID.setSetpoint(targetAngle);
    }


    public void resetEncoder () {
        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Returns the current yaw angle of the module, in degrees (wrapped between 0 and 360).
     */

    public void aligned(double speed){
        driveMotor.setPower(invertedDrive ? -speed : speed);
    }

    public double getCurrentAngle() {
        return Utils.wrapAngle(yawEncoder.getCurrentPosition() / ticksPerDegree);
    }

    /**
     * Returns the target angle that the module is trying to achieve.
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Returns the current yaw error (in degrees).
     */
    public double getYawError() {
        return yawError;
    }


    /**
     * Returns the current drive power setting.
     */
    public double getDrivePowerActual() {
        return driveMotor.getPower();
    }
    /**
     * Returns the current drive power setting.
     */
    public double getDriveAmps() {
        return driveMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getDesiredAngle() {
        return desiredAnglePrivy;
    }

    public void resetEncoders() {
        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yawEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Object getDriveMotor() {
        return driveMotor;
    }

    public DcMotorEx getYawEncoder() {
        return yawEncoder;
    }

    public double getYawAnalog() {
        return yawAnalog.getVoltage();
    }

    public void startCalibration()        { calibKind = CalibKind.NORMAL;  calibState = CalibState.SEEK_FAST; resetScan(); }
    public void startTickCharacterisation()    { calibKind = CalibKind.TICK_CHAR; calibState = CalibState.ONE_REV_START; resetScan(); }
    public void startOffsetCharacterisation()  { calibKind = CalibKind.OFFSET_CHAR; calibState = CalibState.SEEK_FAST; resetScan(); }
    public boolean isCalibrationDone()         { return calibState == CalibState.DONE; }

    private void resetScan() {
        scanTicks   = 0;
        analogMin   =  1e9;
        analogMax   = -1e9;
    }

    /**
     * Call every loop() *before* normal driving so the steer motor is free.
     * @return true once the chosen calibration routine completes
     */
    public boolean calibrate() {

        switch (calibKind) {

            case NONE: return true; // no calibration or characterization to be done

            // ---------- DAILY CALIBRATION ----------------------------
            case NORMAL:
                switch (calibState) {

                    case SEEK_FAST: {             // spin CW fast until we hit *either* tape
                        yawServo.setPower(FAST_PWR);
                        if (tapeSeen()) { calibState = CalibState.SEEK_SLOW; }
                        break;
                    }

                    case SEEK_SLOW: {             // keep spinning slowly & gather extremes
                        yawServo.setPower(SLOW_PWR);
                        double v = yawAnalog.getVoltage();
                        if (v < analogMin) analogMin = v;
                        if (v > analogMax) analogMax = v;

                        // after MIN_SCAN_TICS of slow scan assume we have both extremes
                        if (++scanTicks > MIN_SCAN_TICS) {
                            midValue   = (analogMin + analogMax) / 2.0;
                            calibState = CalibState.FIND_EXTREMES;
                        }
                        break;
                    }

                    case FIND_EXTREMES: {         // keep moving until we *leave* the boundary
                        double v = yawAnalog.getVoltage();
                        yawServo.setPower(SLOW_PWR);
                        if (Math.abs(v - midValue) < ANALOG_HYST) {
                            // still on the boundary, carry on
                        } else {
                            // we just left – now decide which side we exited
                            calibState = (v > midValue) ?
                                    CalibState.CENTER_FROM_POS : CalibState.CENTER_FROM_NEG;
                        }
                        break;
                    }

                    case CENTER_FROM_POS:         // approach boundary always CW so backlash is same
                    case CENTER_FROM_NEG: {
                        double v = yawAnalog.getVoltage();
                        yawServo.setPower(calibState == CalibState.CENTER_FROM_POS ? SLOW_PWR : -SLOW_PWR);
                        if (Math.abs(v - midValue) < ANALOG_HYST) {
                            yawServo.setPower(0);
                            calibState = CalibState.TEMP_ZERO;
                        }
                        break;
                    }

                    case TEMP_ZERO: {             // zero encoder while exactly on boundary
                        resetEncoders();          // STOP_AND_RESET + RUN_WITHOUT
                        calibState = CalibState.GOTO_OFFSET;
                        yawPID.reset();           // (optional) clear integral
                        yawPID.setSetpoint(alignmentOffsetDeg);
                        break;
                    }

                    case GOTO_OFFSET: {           // use PID to drive to real-world align
                        double current = getCurrentAngle();
                        double error   = utilMethods.angleDifference(alignmentOffsetDeg, current);
                        double out     = yawPID.performPID();
                        yawServo.setPower(-out);

                        if (Math.abs(error) < OFFSET_EPS) {
                            yawServo.setPower(0);
                            resetEncoders();      // this is the “final” zero
                            calibState = CalibState.DONE;
                        }
                        break;
                    }

                    case DONE: {
                        calibKind = CalibKind.NONE;
                        return true;
                    }
                }
                break;

            // ---------- TICK CHARACTERISATION (single revolution) -----
            case TICK_CHAR:
                switch (calibState) {

                    case ONE_REV_START: {
                        resetEncoders();
                        startAngle = getCurrentAngle();
                        yawServo.setPower(FAST_PWR);
                        calibState = CalibState.ONE_REV_COUNT;
                        break;
                    }

                    case ONE_REV_COUNT: {
                        if (Math.abs(utilMethods.angleDifference(getCurrentAngle(), startAngle)) > 350) {
                            yawServo.setPower(0);
                            revTicks   = yawEncoder.getCurrentPosition();
                            calibState = CalibState.BACKLASH_FWD;
                            resetEncoders();
                        }
                        break;
                    }

                    case BACKLASH_FWD: {
                        yawServo.setPower(SLOW_PWR);
                        if (tapeSeen()) {          // saw boundary going fwd
                            yawServo.setPower(0);
                            resetEncoders();
                            calibState = CalibState.BACKLASH_REV;
                        }
                        break;
                    }

                    case BACKLASH_REV: {
                        yawServo.setPower(-SLOW_PWR);
                        if (tapeSeen()) {          // saw boundary from opposite dir
                            yawServo.setPower(0);
                            backlashTicks = Math.abs(yawEncoder.getCurrentPosition());
                            calibState    = CalibState.DONE;
                        }
                        break;
                    }

                    case DONE: {
                        // copy revTicks & backlashTicks somewhere (telemetry)
                        calibKind = CalibKind.NONE;
                        return true;
                    }
                }
                break;

            // ---------- OFFSET CHARACTERISATION -----------------------
            case OFFSET_CHAR:
                // identical to NORMAL calibration but we *stop* at TEMP_ZERO and wait for operator
                if (calibState == CalibState.TEMP_ZERO) {
                    yawServo.setPower(0);
                    // Operator aligns wheels by hand, then captures yawEncoder pos
                    calibState = CalibState.DONE;
                } else if (calibState == CalibState.DONE) {
                    calibKind = CalibKind.NONE;
                    return true;
                } else {
                    // reuse the same logic as DAILY
                    calibKind = CalibKind.NORMAL;
                    calibrate();   // recursion one level deep is fine here
                }
                break;

            default: break;
        }

        return false;      // still calibrating
    }

    /** True if sensor is over either tape (outside “normal” material). */
    private boolean tapeSeen() {
        double v = yawAnalog.getVoltage();
        return (v < analogMin + ANALOG_HYST) || (v > analogMax - ANALOG_HYST);
    }

    public int    getRevTicks()     { return revTicks; }
    public double getBacklashTicks(){ return backlashTicks; }

}
