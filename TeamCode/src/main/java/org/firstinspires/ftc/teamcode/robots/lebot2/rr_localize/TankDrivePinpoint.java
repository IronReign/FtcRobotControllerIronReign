package org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize;

//import static org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.PinpointLocalizer.goBILDA_4_BAR_POD;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.drivetrain.DriveTrainBase;
import org.firstinspires.ftc.teamcode.robots.lebot2.subsystem.Vision;
import org.firstinspires.ftc.teamcode.rrQuickStart.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.rrQuickStart.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.rrQuickStart.messages.TankCommandMessage;
import org.firstinspires.ftc.teamcode.rrQuickStart.Drawing;
import org.firstinspires.ftc.teamcode.rrQuickStart.Localizer;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * TankDrive using Pinpoint odometry, implementing DriveTrainBase for Subsystem integration.
 *
 * This class serves dual purposes:
 * 1. RoadRunner trajectory following (Actions, kinematics, feedforward)
 * 2. Subsystem integration (three-phase updates, teleop driving, PID turns)
 *
 * THREE-PHASE UPDATE:
 * - readSensors(): Refresh Pinpoint localizer (I2C bulk read)
 * - calc(): Update pose estimate, run turn PID if active
 * - act(): No-op (RoadRunner Actions write motors directly)
 *
 * TUNING COMPATIBILITY:
 * Public fields (leftMotors, rightMotors, localizer, PARAMS) are preserved
 * for TuningOpModes compatibility.
 */
@Config(value = "Lebot2_TankDrivePinpoint")
public final class TankDrivePinpoint implements DriveTrainBase {

    // ==================== ROADRUNNER PARAMS (for tuning compatibility) ====================

    public static class Params {
        // drive model parameters
        public double inPerTick = 19.89436789f * 25.4;  //23.75/11995; // Pinpoint reports in inches, so 1:1 mapping
        public double trackWidthTicks = 4777.861530266601; // Track width in inches for kinematics         //14

        // feedforward parameters (in tick units)
        public double kS = 1.164863007541458;
        public double kV = 0.0003462964757510799;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double ramseteZeta = 0.7; // in the range (0, 1)
        public double ramseteBBar = 2.0; // positive

        // turn controller gains
        public double turnGain = 0.0;
        public double turnVelGain = 0.0;
    }

    public static Params PARAMS = new Params();

    // ==================== TURN PID PARAMS ====================

    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0.03, 0.04, 0.0);
    public static double HEADING_TOLERANCE = 2.0; // degrees

    // ==================== ROADRUNNER INFRASTRUCTURE ====================

    public final TankKinematics kinematics;

    public final TurnConstraints defaultTurnConstraints;
    public final VelConstraint defaultVelConstraint;
    public final AccelConstraint defaultAccelConstraint;

    // Public for TuningOpModes compatibility
    public final List<DcMotorEx> leftMotors, rightMotors;
    public final VoltageSensor voltageSensor;
    public final Localizer localizer;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter tankCommandWriter = new DownsampledWriter("TANK_COMMAND", 50_000_000);

    // ==================== SUBSYSTEM STATE ====================

    // Drive behavior - what is currently controlling the drive motors
    // Set implicitly by control methods (drive, runAction, turnToHeading)
    public enum Behavior {
        MANUAL,     // Joystick control (default)
        TRAJECTORY, // RoadRunner trajectory following
        PID_TURN    // PID-based turn to heading/target
    }
    private Behavior behavior = Behavior.MANUAL;

    // Turn state machine (used when behavior == PID_TURN)
    public enum TurnState {
        IDLE,
        TURNING_TO_HEADING,
        TURNING_TO_TARGET,      // Single tx value (legacy)
        CENTERING_ON_TARGET     // Continuous tx from Vision (preferred)
    }
    private TurnState turnState = TurnState.IDLE;
    private double turnTarget = 0;
    private double turnMaxSpeed = 1.0;

    // Vision reference for continuous target tracking
    private Vision vision = null;
    public static double CENTERING_MAX_SPEED = 0.5; // Default max speed for centering

    // RoadRunner action tracking
    private Action currentAction = null;

    // PID controller for turns
    private final PIDController headingPID;

    // Cached values from readSensors()
    private double cachedHeading = 0;
    private Pose2d cachedPose = new Pose2d(0, 0, 0);

    // ==================== CONSTRUCTOR ====================

    public TankDrivePinpoint(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        // NOTE: Bulk caching mode is NOT set here - Robot.java handles it
        // This allows TuningOpModes to set their own mode if needed

        // Initialize kinematics with track width
        //kinematics = new TankKinematics(PARAMS.inPerTick * PARAMS.trackWidthTicks);
        kinematics = new TankKinematics(14);        //track width of robot is 14 inches from center to center of wheels
        // Initialize constraints
        defaultTurnConstraints = new TurnConstraints(
                PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
        defaultVelConstraint = new MinVelConstraint(Arrays.asList(
                kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                new AngularVelConstraint(PARAMS.maxAngVel)
        ));
        defaultAccelConstraint = new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

        // Lebot2 motor configuration - differential drive with rear-mounted motors
        leftMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "leftRear"));
        rightMotors = Arrays.asList(hardwareMap.get(DcMotorEx.class, "rightRear"));

        for (DcMotorEx m : leftMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotorEx m : rightMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Lebot2: left motor reversed so positive power = forward
        rightMotors.get(0).setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Use Pinpoint for localization
        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose);

        // Initialize heading PID for turns
        headingPID = new PIDController(HEADING_PID);
        headingPID.setInputRange(-180, 180);
        headingPID.setOutputRange(-1, 1);
        headingPID.setIntegralCutIn(4);
        headingPID.setContinuous(true);
        headingPID.setTolerance(HEADING_TOLERANCE);
        headingPID.enable();

        cachedPose = pose;
        cachedHeading = Math.toDegrees(pose.heading.toDouble());

        FlightRecorder.write("TANK_PINPOINT_PARAMS", PARAMS);
    }

    // ==================== THREE-PHASE SUBSYSTEM METHODS ====================

    @Override
    public void readSensors() {
        // PHASE 1: Refresh Pinpoint localizer (triggers I2C bulk read)
        if (localizer instanceof PinpointLocalizer) {
            ((PinpointLocalizer) localizer).refresh();
        }
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        // PHASE 2: Update pose estimate and run turn state machine

        // Get pose from localizer (uses cached Pinpoint data)
        PoseVelocity2d vel = localizer.update();
        cachedPose = localizer.getPose();
        cachedHeading = Math.toDegrees(cachedPose.heading.toDouble());

        // Update pose history for visualization
        poseHistory.add(cachedPose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }
        estimatedPoseWriter.write(new PoseMessage(cachedPose));

        // Run turn state machine
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

            case CENTERING_ON_TARGET:
                executeCenteringOnTarget();
                break;
        }

        // Draw robot on field overlay if provided
        if (fieldOverlay != null) {
            drawPoseHistory(fieldOverlay);
            fieldOverlay.setStroke("#3F51B5");
            Drawing.drawRobot(fieldOverlay, cachedPose);
        }
    }

    @Override
    public void act() {
        // PHASE 3: No-op for drive motors
        // RoadRunner Actions write motors directly during trajectory following
        // Turn PID writes motors in calc() for immediate response
    }

    // ==================== TURN METHODS ====================

    private void executeTurnToHeading() {
        headingPID.setInput(cachedHeading);
        headingPID.setSetpoint(turnTarget);
        headingPID.setOutputRange(-turnMaxSpeed, turnMaxSpeed);
        headingPID.setPID(HEADING_PID);

        double correction = headingPID.performPID();

        if (headingPID.onTarget()) {
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
            behavior = Behavior.MANUAL;
        } else {
            // Apply turn power (positive = clockwise)
            setMotorPowers(correction, -correction);
        }
    }

    private void executeTurnToTarget() {
        // For vision-based turning, we want tx (turnTarget) to reach 0
        // Input = current offset, Setpoint = desired offset (0)
        headingPID.setInput(turnTarget);
        headingPID.setSetpoint(0);
        headingPID.setOutputRange(-turnMaxSpeed, turnMaxSpeed);
        headingPID.setPID(HEADING_PID);

        double correction = headingPID.performPID();

        if (headingPID.onTarget()) {
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
            behavior = Behavior.MANUAL;
        } else {
            setMotorPowers(correction, -correction);
        }
    }

    @Override
    public void turnToHeading(double headingDegrees, double maxSpeed) {
        turnTarget = headingDegrees;
        turnMaxSpeed = maxSpeed;
        turnState = TurnState.TURNING_TO_HEADING;
        behavior = Behavior.PID_TURN;
        headingPID.enable();
    }

    @Override
    public void turnToTarget(double tx, double maxSpeed) {
        turnTarget = tx; // tx is the offset, we want to drive it to 0
        turnMaxSpeed = maxSpeed;
        turnState = TurnState.TURNING_TO_TARGET;
        behavior = Behavior.PID_TURN;
        headingPID.enable();
    }

    @Override
    public boolean isTurnComplete() {
        return turnState == TurnState.IDLE;
    }

    @Override
    public void cancelTurn() {
        turnState = TurnState.IDLE;
        if (behavior == Behavior.PID_TURN) {
            behavior = Behavior.MANUAL;
        }
        setMotorPowers(0, 0);
    }

    @Override
    public void setVision(Vision vision) {
        this.vision = vision;
    }

    @Override
    public void centerOnTarget() {
        // Only start if we have Vision reference and target is visible
        if (vision == null) {
            return; // No vision reference - can't center
        }
        if (!vision.hasTarget()) {
            return; // No target visible - do nothing
        }

        // Start centering - will query Vision each loop for fresh tx
        turnMaxSpeed = CENTERING_MAX_SPEED;
        turnState = TurnState.CENTERING_ON_TARGET;
        behavior = Behavior.PID_TURN;
        headingPID.enable();
    }

    private void executeCenteringOnTarget() {
        // Query Vision for fresh tx each loop
        if (vision == null || !vision.hasTarget()) {
            // Lost target or no vision - stop and return to manual
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
            behavior = Behavior.MANUAL;
            return;
        }

        double tx = vision.getTx();

        // PID: we want tx to be 0 (target centered)
        // Input = current offset, Setpoint = desired offset (0)
        headingPID.setInput(tx);
        headingPID.setSetpoint(0);
        headingPID.setOutputRange(-turnMaxSpeed, turnMaxSpeed);
        headingPID.setPID(HEADING_PID);

        double correction = headingPID.performPID();

        if (headingPID.onTarget()) {
            setMotorPowers(0, 0);
            turnState = TurnState.IDLE;
            behavior = Behavior.MANUAL;
        } else {
            setMotorPowers(correction, -correction);
        }
    }

    // ==================== TELEOP DRIVING ====================

    @Override
    public void drive(double throttle, double strafe, double turn) {
        boolean seperate = false;
        if(seperate){
            setMotorPowers(throttle, strafe);
            return;
        }else {

            // Tank drive ignores strafe

            // Check if joystick input should interrupt current mode
            boolean hasInput = Math.abs(throttle) > 0.1 || Math.abs(turn) > 0.1;

            if (hasInput) {
                // Joystick input interrupts RR_ACTION and PID_TURN
                if (behavior == Behavior.TRAJECTORY) {
                    cancelAction();
                }
                if (behavior == Behavior.PID_TURN) {
                    cancelTurn();
                }
                behavior = Behavior.MANUAL;
            }

            // Only apply joystick input in MANUAL mode
            if (behavior != Behavior.MANUAL) {
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
    }

    // ==================== BEHAVIOR MANAGEMENT ====================

    /**
     * Get current drive behavior.
     */
    public Behavior getBehavior() {
        return behavior;
    }

    /**
     * Check if drive is in manual mode.
     */
    public boolean isManualMode() {
        return behavior == Behavior.MANUAL;
    }

    /**
     * Check if a RoadRunner action is currently running.
     */
    public boolean isActionRunning() {
        return behavior == Behavior.TRAJECTORY && currentAction != null;
    }

    /**
     * Run a RoadRunner action. Sets behavior to TRAJECTORY.
     *
     * @param action The action to run
     */
    public void runAction(Action action) {
        currentAction = action;
        behavior = Behavior.TRAJECTORY;
    }

    /**
     * Cancel any running RoadRunner action.
     */
    public void cancelAction() {
        currentAction = null;
        if (behavior == Behavior.TRAJECTORY) {
            behavior = Behavior.MANUAL;
            setMotorPowers(0, 0);
        }
    }

    /**
     * Update the current RoadRunner action.
     * Call this in the main loop when an action is running.
     *
     * @param packet TelemetryPacket for logging
     * @return true if action is still running, false if complete
     */
    public boolean updateAction(TelemetryPacket packet) {
        if (currentAction == null) {
            return false;
        }

        boolean running = currentAction.run(packet);
        if (!running) {
            currentAction = null;
            behavior = Behavior.MANUAL;
        }
        return running;
    }

    /**
     * Set drive powers using RoadRunner's PoseVelocity2d format.
     * Used by RoadRunner Actions during trajectory following.
     */
    public void setDrivePowers(PoseVelocity2d powers) {
        TankKinematics.WheelVelocities<Time> wheelVels = new TankKinematics(2).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        for (DcMotorEx m : leftMotors) {
            m.setPower(wheelVels.left.get(0) / maxPowerMag);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(wheelVels.right.get(0) / maxPowerMag);
        }
    }

    private void setMotorPowers(double left, double right) {
        for (DcMotorEx m : leftMotors) {
            m.setPower(left);
        }
        for (DcMotorEx m : rightMotors) {
            m.setPower(right);
        }
    }

    // ==================== POSE METHODS ====================

    @Override
    public Pose2d getPose() {
        return cachedPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        if (localizer instanceof PinpointLocalizer) {
            ((PinpointLocalizer) localizer).setPose(pose);
        }
        cachedPose = pose;
        cachedHeading = Math.toDegrees(pose.heading.toDouble());
    }

    @Override
    public void setPose(Object position) {
        // TODO: Convert position constant to Pose2d and set
        // This will be implemented when we have the Constants class
    }

    @Override
    public double getHeadingDegrees() {
        return cachedHeading;
    }

    @Override
    public int getLeftTicks() {
        return leftMotors.get(0).getCurrentPosition();
    }

    @Override
    public void resetEncoders() {
        for (DcMotorEx m : leftMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for (DcMotorEx m : rightMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // ==================== ROADRUNNER POSE ESTIMATE ====================

    /**
     * Update pose estimate from localizer.
     * Called by RoadRunner Actions during trajectory following.
     */
    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        cachedPose = localizer.getPose();
        cachedHeading = Math.toDegrees(cachedPose.heading.toDouble());

        poseHistory.add(cachedPose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(cachedPose));

        return vel;
    }

    // ==================== ROADRUNNER ACTIONS ====================

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                setMotorPowers(0, 0);
                return false;
            }

            DualNum<Time> x = timeTrajectory.profile.get(t);

            Pose2dDual<Arclength> txWorldTarget = timeTrajectory.path.get(x.value(), 3);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new RamseteController(kinematics.trackWidth, PARAMS.ramseteZeta, PARAMS.ramseteBBar)
                    .compute(x, txWorldTarget, localizer.getPose());
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            setMotorPowers(leftPower, rightPower);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                setMotorPowers(0, 0);
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new PoseVelocity2dDual<>(
                    Vector2dDual.constant(new Vector2d(0, 0), 3),
                    txWorldTarget.heading.velocity().plus(
                            PARAMS.turnGain * localizer.getPose().heading.minus(txWorldTarget.heading.value()) +
                                    PARAMS.turnVelGain * (robotVelRobot.angVel - txWorldTarget.heading.velocity().value())
                    )
            );
            driveCommandWriter.write(new DriveCommandMessage(command));

            TankKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double leftPower = feedforward.compute(wheelVels.left) / voltage;
            double rightPower = feedforward.compute(wheelVels.right) / voltage;
            tankCommandWriter.write(new TankCommandMessage(voltage, leftPower, rightPower));

            setMotorPowers(leftPower, rightPower);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;
            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    // ==================== SUBSYSTEM LIFECYCLE ====================

    @Override
    public void stop() {
        setMotorPowers(0, 0);
        turnState = TurnState.IDLE;
        currentAction = null;
        behavior = Behavior.MANUAL;
    }

    @Override
    public void resetStates() {
        turnState = TurnState.IDLE;
        currentAction = null;
        behavior = Behavior.MANUAL;
    }

    // ==================== TELEMETRY ====================

    @Override
    public String getTelemetryName() {
        return "TankDrivePinpoint";
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Drive Mode", behavior);
        telemetry.put("Heading (deg)", String.format("%.1f", cachedHeading));
        telemetry.put("Pose", String.format("(%.1f, %.1f)", cachedPose.position.x, cachedPose.position.y));

        if (debug) {
            telemetry.put("Turn State", turnState);
            telemetry.put("Turn Target", turnTarget);
            telemetry.put("PID Error", headingPID.getError());
            telemetry.put("Left Power", leftMotors.get(0).getPower());
            telemetry.put("Right Power", rightMotors.get(0).getPower());
            telemetry.put("Action Running", currentAction != null);
        }

        return telemetry;
    }
}
