package org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.drivetrain;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.LinkedHashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;

/**
 * Tank (differential) drive implementation.
 *
 * This drivetrain has two powered wheels at the rear and passive omni wheels
 * at the front. It cannot strafe, so the strafe parameter is ignored.
 *
 * Uses the Pinpoint localizer for accurate odometry instead of motor encoders.
 */
@Config(value = "Lebot2_TankDrive")
public class TankDrive implements DriveTrainBase {

    // Hardware
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final BNO055IMU imu;
    // TODO: Add Pinpoint localizer when hardware is ready
    // private final PinpointLocalizer localizer;

    // PID for heading control
    private final PIDController headingPID;
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.03, 0.04, 0.0);
    public static double HEADING_TOLERANCE = 2.0; // degrees

    // Turn state
    public enum TurnState {
        IDLE,
        TURNING_TO_HEADING,
        TURNING_TO_TARGET
    }
    private TurnState turnState = TurnState.IDLE;
    private double turnTarget = 0;
    private double turnMaxSpeed = 1.0;

    // Current pose (will be replaced by localizer)
    private Pose2d pose = new Pose2d(0, 0, 0);

    public TankDrive(HardwareMap hardwareMap) {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        // Initialize heading PID
        headingPID = new PIDController(HEADING_PID);
        headingPID.setInputRange(0, 360);
        headingPID.setOutputRange(-1, 1);
        headingPID.setIntegralCutIn(4);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_TOLERANCE);
        headingPID.enable();

        // TODO: Initialize Pinpoint localizer
        // localizer = new PinpointLocalizer(hardwareMap, IN_PER_TICK, pose);
    }

    @Override
    public void update(Canvas fieldOverlay) {
        // Update localizer
        // TODO: localizer.update();
        // pose = localizer.getPose();

        // Handle turning state machine
        switch (turnState) {
            case IDLE:
                // Nothing to do
                break;

            case TURNING_TO_HEADING:
                executeTurnToHeading();
                break;

            case TURNING_TO_TARGET:
                executeTurnToTarget();
                break;
        }
    }

    private void executeTurnToHeading() {
        double currentHeading = getHeadingDegrees();
        headingPID.setInput(wrapAngle(currentHeading));
        headingPID.setSetpoint(wrapAngle(turnTarget));
        headingPID.setOutputRange(-turnMaxSpeed, turnMaxSpeed);
        headingPID.setPID(HEADING_PID);

        double correction = headingPID.performPID();

        if (headingPID.onTarget()) {
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
        } else {
            // Apply turn power (positive = clockwise)
            setMotorPowers(correction, -correction);
        }
    }

    private void executeTurnToTarget() {
        // For vision-based turning, tx IS the error (0 = centered)
        headingPID.setInput(0); // We want tx to be 0
        headingPID.setSetpoint(turnTarget); // turnTarget holds the tx value
        headingPID.setOutputRange(-turnMaxSpeed, turnMaxSpeed);

        double correction = headingPID.performPID();

        if (headingPID.onTarget()) {
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
        } else {
            setMotorPowers(correction, -correction);
        }
    }

    @Override
    public void drive(double throttle, double strafe, double turn) {
        // Tank drive ignores strafe
        if (turnState != TurnState.IDLE) {
            // Don't override an active turn
            return;
        }

        double leftPower = throttle + turn;
        double rightPower = throttle - turn;

        // Normalize if over 1.0
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        setMotorPowers(leftPower, rightPower);
    }

    private void setMotorPowers(double left, double right) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
    }
    
    @Override
    public void turnToHeading(double headingDegrees, double maxSpeed) {
        turnTarget = headingDegrees;
        turnMaxSpeed = maxSpeed;
        turnState = TurnState.TURNING_TO_HEADING;
        headingPID.enable();
    }

    @Override
    public void turnToTarget(double tx, double maxSpeed) {
        turnTarget = tx; // tx is the offset, we want to drive it to 0
        turnMaxSpeed = maxSpeed;
        turnState = TurnState.TURNING_TO_TARGET;
        headingPID.enable();
    }

    @Override
    public boolean isTurnComplete() {
        return turnState == TurnState.IDLE;
    }

    @Override
    public void cancelTurn() {
        turnState = TurnState.IDLE;
        setMotorPowers(0, 0);
    }

    public int getLeftTicks(){
        return leftMotor.getCurrentPosition();
    }

    @Override
    public Pose2d getPose() {
        // TODO: return localizer.getPose();
        return pose;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
        // TODO: localizer.setPose(pose);
    }

    @Override
    public void setPose(Object position) {
        // TODO: Convert position constant to Pose2d and set
        // This will be implemented when we have the Constants class
    }

    @Override
    public double getHeadingDegrees() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    @Override
    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void stop() {
        setMotorPowers(0, 0);
        turnState = TurnState.IDLE;
    }

    @Override
    public void resetStates() {
        turnState = TurnState.IDLE;
    }

    @Override
    public String getTelemetryName() {
        return "TankDrive";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Turn State", turnState);
        telemetry.put("Heading (deg)", String.format("%.1f", getHeadingDegrees()));

        if (debug) {
            telemetry.put("Turn Target", turnTarget);
            telemetry.put("PID Error", headingPID.getError());
            telemetry.put("Left Power", leftMotor.getPower());
            telemetry.put("Right Power", rightMotor.getPower());
            telemetry.put("Pose", pose.toString());
        }

        return telemetry;
    }
}
