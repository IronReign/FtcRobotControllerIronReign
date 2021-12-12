package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.MathUtils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.util.utilMethods.wrap360;

public class DriveTrain implements Subsystem {

    // Motors
    private DcMotorEx motorFrontLeft, motorFrontRight, motorMiddle, motorMiddleSwivel, duckSpinner;
    private DcMotorEx[] motors;
    private String[] MOTOR_NAMES = {"motorFrontLeft", "motorFrontRight", "motorMiddle", "motorMiddleSwivel", "duckSpinner"};
    private boolean[] REVERSED = {true, false, true, false, false};
    private DcMotor.ZeroPowerBehavior[] ZERO_POWER_BEHAVIORS = new DcMotor.ZeroPowerBehavior[] {ZeroPowerBehavior.FLOAT, ZeroPowerBehavior.FLOAT, ZeroPowerBehavior.FLOAT, ZeroPowerBehavior.BRAKE, ZeroPowerBehavior.BRAKE};

    // Sensors
    private BNO055IMU imu;
    private DistanceSensor sensorChassisDistance;

    // Kinematics
    private SimpleMatrix pose; // [x, y, yaw]
    private SimpleMatrix velocity; // [vx, vy, w]
    private SimpleMatrix angles; // [heading, roll, pitch]
    private SimpleMatrix offsetAngles; // [heading, roll, pitch]
    private SimpleMatrix previousWheelTicks; // [left, right, middle]

    // PIVs
    private double targetFrontLeftVelocity, targetFrontRightVelocity, targetMiddleVelocity, targetSwivelAngle;
    private double targetLinearVelocity, targetAngularVelocity, targetTurnRadius;

    private double swivelAngle;
    private double chassisDistance, targetChassisDistance;

    // PID
    private PIDController turnPID, drivePID, distPID, swivelPID, chassisDistancePID;
    private double maintainSwivelAngleCorrection, maintainChassisDistanceCorrection;
    private boolean maintainChassisDistanceEnabled;

    // Smoothers
    private ExponentialSmoother frontLeftSmoother;
    private ExponentialSmoother frontRightSmoother;
    private ExponentialSmoother middleSmoother;

    // Constants
    public static final String TELEMETRY_NAME = "Drive Train";

    public DriveTrain(HardwareMap hardwareMap) {
        // Motors
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorMiddle= hardwareMap.get(DcMotorEx.class, "motorMiddle");
        motorMiddleSwivel = hardwareMap.get(DcMotorEx.class, "motorMiddleSwivel");
        duckSpinner = hardwareMap.get(DcMotorEx.class,"duckSpinner");
        motors = new DcMotorEx[] {motorFrontLeft, motorFrontRight, motorMiddle, motorMiddleSwivel, duckSpinner};

        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MOTOR_NAMES[i]);
            motors[i].setMode(RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(ZERO_POWER_BEHAVIORS[i]);
            if(REVERSED[i])
                motors[i].setDirection(Direction.REVERSE);
        }

        // Sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        initializeIMU();

        sensorChassisDistance = hardwareMap.get(DistanceSensor.class, "distLength");

        // Kinematics
        pose = new SimpleMatrix(3, 1);
        velocity = new SimpleMatrix(3, 1);
        angles = new SimpleMatrix(3, 1);
        offsetAngles = new SimpleMatrix(3, 1);
        previousWheelTicks = new SimpleMatrix(3, 2);

        // PID
        turnPID = new PIDController(Constants.ROTATE_PID_COEFFICIENTS);
        drivePID = new PIDController(Constants.DRIVE_PID_COEFFICIENTS);
        swivelPID = new PIDController(Constants.SWIVEL_PID_COEFFICIENTS);
        distPID = new PIDController(Constants.DIST_PID_COEFFICIENTS);
        chassisDistancePID = new PIDController(Constants.CHASSIS_DISTANCE_PID_COEFFICIENTS);

        // Smoother
        frontLeftSmoother = new ExponentialSmoother(Constants.FRONT_LEFT_SMOOTHING_FACTOR);
        frontRightSmoother = new ExponentialSmoother(Constants.FRONT_RIGHT_SMOOTHING_FACTOR);
        middleSmoother = new ExponentialSmoother(Constants.MIDDLE_SMOOTHING_FACTOR);

        // Miscellaneous
        previousWheelTicks = getWheelTicks();
    }

    private void initializeIMU() {
        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters();
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersIMU.loggingEnabled = true;
        parametersIMU.loggingTag = "baseIMU";

        imu.initialize(parametersIMU);

        // storing first absolute orientation values as offsets
        Orientation imuAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        offsetAngles = new SimpleMatrix(new double[][] {
                { (360 - imuAngles.firstAngle) % 360 },
                { imuAngles.secondAngle % 360 },
                { imuAngles.thirdAngle % 360 }
        });
    }

    private double getMaintainSwivelAngleCorrection() {
        //initialization of the PID calculator's output range, target value and multipliers
        swivelPID.setOutputRange(-1.0, 1.0);
        swivelPID.setPID(Constants.SWIVEL_PID_COEFFICIENTS);
        swivelPID.setSetpoint(targetSwivelAngle);
        swivelPID.setTolerance(Constants.SWIVEL_PID_TOLERANCE);
        swivelPID.enable();

        //initialization of the PID calculator's input range and current value
        swivelPID.setInputRange(0, 2 * Math.PI);
        swivelPID.setContinuous(true);
        swivelPID.setInput(swivelAngle);

        //calculates the angular correction to apply
        return swivelPID.performPID();
    }

    /**
     * updates the target chassis distance to maintain rotation without slipping
     */
    private void updateTargetChassisDistance() {
        double turnRadius = targetAngularVelocity == 0 ? 0 : targetLinearVelocity / targetAngularVelocity;
        targetChassisDistance = targetAngularVelocity == 0 ?
            // if not rotating, set target chassis distance to max length - threshold
            Constants.MAX_CHASSIS_LENGTH - Constants.CHASSIS_LENGTH_THRESHOLD :
            // when rotating, if calculated distance is smaller than minimum length, set to minimum length
            Math.max(
                Constants.MIN_CHASSIS_LENGTH + Constants.CHASSIS_LENGTH_THRESHOLD,
                // when rotating, if calculated distance is greater than maximum length, set to maximum length
                Math.min(
                    Constants.MAX_CHASSIS_LENGTH - Constants.CHASSIS_LENGTH_THRESHOLD,
                    Math.sqrt(
                        Math.pow(Constants.DRIVETRAIN_COEFFICIENT_OF_FRICTION * Constants.ACCELERATION_OF_GRAVITY / Math.pow(targetAngularVelocity, 2), 2)
                      - Math.pow(turnRadius, 2)
                )
            )
        );
    }

    private double getMaintainChassisDistanceCorrection() {
        // initialization of the PID calculator's output range, target value and multipliers
        chassisDistancePID.setOutputRange(-5.0, 5.0);
        chassisDistancePID.setPID(Constants.CHASSIS_DISTANCE_PID_COEFFICIENTS);
        chassisDistancePID.setSetpoint(targetChassisDistance);
        chassisDistancePID.enable();

        // initialization of the PID calculator's input range and current value
        chassisDistancePID.setInputRange(0, Constants.MAX_CHASSIS_LENGTH);
        chassisDistancePID.setInput(chassisDistance);

        // calculating correction
        return chassisDistancePID.performPID();
    }

    /**
     * updates the robot's pose ((x,y) position and heading) using the encoder ticks travelled by each wheel motor
     */
    private void updatePose() {
        Orientation imuAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        angles = new SimpleMatrix(new double[][] {
                {MathUtils.wrapAngle(360 - imuAngles.firstAngle, offsetAngles.get(0))},
                {MathUtils.wrapAngle(imuAngles.thirdAngle, offsetAngles.get(1))},
                {MathUtils.wrapAngle(imuAngles.secondAngle, offsetAngles.get(2))}
        });

        // calculating wheel displacements
//        SimpleMatrix wheelTicks = getWheelTicks().rows(0, 2);
//        SimpleMatrix wheelDisplacementMeters = wheelTicks.minus(previousWheelTicks.rows(0, 2)).divide(Constants.DRIVETRAIN_TICKS_PER_METER);

//        // rotating swivel wheel by swivel angle
//        double swivelAngle = getSwivelAngle();
//        SimpleMatrix swivelWheel = MathUtil.rotateVector(
//                new SimpleMatrix(
//                        new double[][] {{ wheelDisplacementMeters.get(2, 0), 0 }}
//                ),
//                swivelAngle
//        );
//        wheelDisplacementMeters.setRow(2, 0, swivelWheel.get(0), swivelWheel.get(1));

        // calculating average average wheel displacement
//        SimpleMatrix ones = new SimpleMatrix(new double[][] {{1, 1}});
//        SimpleMatrix averageDisplacementMeters = ones.mult(wheelDisplacementMeters).divide(2);

        // rotating displacement by heading
        double heading = angles.get(0);
//        averageDisplacementMeters = MathUtils.rotateVector(averageDisplacementMeters, heading);

        // updating pose [x, y, heading]
//        pose = pose.plus(new SimpleMatrix(new double[][] {{
//            averageDisplacementMeters.get(0, 0),
//            averageDisplacementMeters.get(1, 0),
//            0
//        }}).transpose());
        pose.set(2, 0, angles.get(0));

//        previousWheelTicks = wheelTicks.copy();
    }

    @Override
    public void update() {
        // state
        chassisDistance = sensorChassisDistance.getDistance(DistanceUnit.MM) / 1000 + Constants.DISTANCE_SENSOR_TO_FRONT_AXLE + Constants.DISTANCE_TARGET_TO_BACK_WHEEL;
//        chassisDistance = Constants.TEST_CHASSIS_DISTANCE;
        swivelAngle = (motorMiddleSwivel.getCurrentPosition() / Constants.SWERVE_TICKS_PER_REVOLUTION * 2 * Math.PI) % (2 * Math.PI);;

        // PID corrections
        maintainSwivelAngleCorrection = getMaintainSwivelAngleCorrection();
        motorMiddleSwivel.setPower(maintainSwivelAngleCorrection);

//        updateTargetChassisDistance();
        if(maintainChassisDistanceEnabled) {
            maintainChassisDistanceCorrection = getMaintainChassisDistanceCorrection();
            targetFrontLeftVelocity += maintainChassisDistanceCorrection;
            targetFrontRightVelocity += maintainChassisDistanceCorrection;
        }

        if(swivelAngle > Math.PI && swivelAngle < 2 * Math.PI) {
//            targetMiddleVelocity = -targetMiddleVelocity;
        }


        // Motor controls
//        if(swivelPID.onTarget()) {
            motorFrontLeft.setVelocity(targetFrontLeftVelocity * Constants.DRIVETRAIN_TICKS_PER_METER);
            motorFrontRight.setVelocity(targetFrontRightVelocity * Constants.DRIVETRAIN_TICKS_PER_METER);
            motorMiddle.setVelocity(targetMiddleVelocity * Constants.DRIVETRAIN_TICKS_PER_METER);
//        } else {
//            motorFrontLeft.setVelocity(0);
//            motorFrontRight.setVelocity(0);
//            motorMiddle.setVelocity(0);
//        }

//        updatePose();
    }

    private void handleSmoothing() {
        targetFrontLeftVelocity = frontLeftSmoother.update(targetFrontLeftVelocity);
        targetFrontRightVelocity = frontRightSmoother.update(targetFrontLeftVelocity);
        targetMiddleVelocity = middleSmoother.update(targetMiddleVelocity);
    }

    /**
     * Drives the robot with the specified linear and angular velocities
     * @param linearVelocity the velocity, in m/s, to drive the robot
     * @param angularVelocity the angular velocity, in rad/s, to drive the robot
     */
    public void drive(double linearVelocity, double angularVelocity) {
        targetLinearVelocity = linearVelocity;
        targetAngularVelocity = angularVelocity;

        targetTurnRadius = angularVelocity == 0 ? 0 : linearVelocity / angularVelocity;

        targetFrontLeftVelocity = linearVelocity + angularVelocity * (targetTurnRadius - Constants.TRACK_WIDTH / 2);
        targetFrontRightVelocity = linearVelocity + angularVelocity * (targetTurnRadius + Constants.TRACK_WIDTH / 2);
        targetMiddleVelocity = linearVelocity + angularVelocity * Math.hypot(targetTurnRadius, chassisDistance);

        targetSwivelAngle = angularVelocity == 0 || (angularVelocity == 0 && linearVelocity == 0)
                ? Math.PI / 2
                    : linearVelocity == 0
                    ? 0
                : Math.PI / 2 - Math.atan2(chassisDistance, targetTurnRadius);
    }

    public void driveDesmos(double linearVelocity, double angularVelocity, double dt) {
        targetLinearVelocity = linearVelocity;
        targetAngularVelocity = angularVelocity;

        targetTurnRadius = angularVelocity == 0 ? 0 : linearVelocity / angularVelocity;

        SimpleMatrix leftWheel = new SimpleMatrix(new double[][] {{ -Constants.TRACK_WIDTH / 2 , 0 }});
        SimpleMatrix rightWheel = new SimpleMatrix(new double[][] {{ Constants.TRACK_WIDTH / 2, 0 }});
        SimpleMatrix middleWheel = new SimpleMatrix(new double[][] {{ 0, -getChassisDistance() }});

        SimpleMatrix translation = new SimpleMatrix(new double[][] {{ 0, linearVelocity * dt }});

        double heading = pose.get(2);

        SimpleMatrix leftWheelPrime = translation.plus(MathUtils.rotateVector(leftWheel, heading).transpose());
        SimpleMatrix rightWheelPrime = translation.plus(MathUtils.rotateVector(rightWheel, heading).transpose());
        SimpleMatrix middleWheelPrime = translation.plus(MathUtils.rotateVector(middleWheel, heading).transpose());

        targetFrontLeftVelocity = leftWheelPrime.minus(leftWheel).normF() / dt;
        targetFrontRightVelocity = rightWheelPrime.minus(rightWheel).normF() / dt;
        targetMiddleVelocity = middleWheelPrime.minus(middleWheel).normF() / dt;

        targetSwivelAngle = angularVelocity == 0 || (angularVelocity == 0 && linearVelocity == 0)
                ? Math.PI / 2
                : linearVelocity == 0
                ? 0
                : Math.PI / 2 - Math.atan2(chassisDistance, targetTurnRadius);
    }

    public void movePID(double maxPwrFwd, boolean forward, double dist, double currentAngle, double targetAngle) {
        // setup turnPID
        turnPID.setOutputRange(-.5, .5);
        turnPID.setIntegralCutIn(1);
        turnPID.setSetpoint(targetAngle);
        turnPID.setInputRange(0, 360);
        turnPID.setContinuous();
        turnPID.setInput(currentAngle);
        turnPID.enable();

        // setup distPID
        distPID.setOutputRange(-maxPwrFwd, maxPwrFwd);
        distPID.setIntegralCutIn(1);
        distPID.setSetpoint(dist); //trying to get to a zero distance
        distPID.setInput(0);
        distPID.enable();

        // calculate the angular correction to apply
        double turnCorrection = turnPID.performPID();
        // calculate chassis power
        double basePwr = distPID.performPID();
        if (!forward) basePwr *=-1;

        // performs the drive with the correction applied

        drive(basePwr, turnCorrection);
    }

    public boolean driveAbsoluteDistance(double pwr, double targetAngle, boolean forward, double targetMeters, double closeEnoughDist) {
        targetAngle= wrap360(targetAngle);  //this was probably already done but repeated as a safety

        if (Math.abs(targetMeters) > Math.abs(closeEnoughDist)) {
            movePID(pwr, forward, targetMeters,getHeading(),targetAngle);
            return false;
        } // destination achieved
        else {
            stop(); //todo: maybe this should be optional when you are stringing moves together
            return true;
        }
    }

    private long turnTimer = 0;
    private boolean turnTimerInit = false;
    private double minTurnError = 2.0;
    public boolean rotateIMU(double targetAngle, double maxTime) {
        if (!turnTimerInit) { // intiate the timer that the robot will use to cut of the sequence if it takes
            // too long; only happens on the first cycle
            turnTimer = System.nanoTime() + (long) (maxTime * (long) 1e9);
            turnTimerInit = true;
        }
        movePID(1,true,0, getHeading(), targetAngle);
        // threshold of the target
        if(Math.abs(getHeading() - targetAngle) < minTurnError) {
            turnTimerInit = false;
            stop();
            return true;
        }

        if (turnTimer < System.nanoTime()) { // check to see if the robot takes too long to turn within a threshold of
            // the target (e.g. it gets stuck)
            turnTimerInit = false;
            stop();
            return true;
        }
        return false;
    }

    public boolean handleDuckSpinner(double power){
        duckSpinner.setPower(power);
        return true;
    }

    boolean duckSpinnerIsOn = false;
    public boolean handleDuckSpinnerToggle(int mod) {
        if(duckSpinnerIsOn) {
            handleDuckSpinner(0);
            duckSpinnerIsOn = false;
        }
        else{
            handleDuckSpinner(mod * .5);
            duckSpinnerIsOn = true;
        }

        return true;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();
        if(debug) {
            telemetryMap.put("fl position", motorFrontLeft.getCurrentPosition());
            telemetryMap.put("fr position", motorFrontRight.getCurrentPosition());
            telemetryMap.put("middle position", motorMiddle.getCurrentPosition());
            telemetryMap.put("swivel position", motorMiddleSwivel.getCurrentPosition());

            telemetryMap.put("fl velocity", MathUtils.ticksToMeters(motorFrontLeft.getVelocity()));
            telemetryMap.put("fr velocity", MathUtils.ticksToMeters(motorFrontRight.getVelocity()));
            telemetryMap.put("middle velocity", MathUtils.ticksToMeters(motorMiddle.getVelocity()));

            telemetryMap.put("fl target velocity", targetFrontLeftVelocity);
            telemetryMap.put("fr target velocity", targetFrontRightVelocity);
            telemetryMap.put("middle target velocity", targetMiddleVelocity);

            telemetryMap.put("swivel angle", Math.toDegrees(swivelAngle));
            telemetryMap.put("target swivel angle", Math.toDegrees(targetSwivelAngle));
            telemetryMap.put("swivel PID on target", swivelPID.onTarget());
            telemetryMap.put("maintain swivel angle correction", maintainSwivelAngleCorrection);

            telemetryMap.put("chassis distance", chassisDistance);
            telemetryMap.put("target chassis distance", targetChassisDistance);
            telemetryMap.put("maintain chassis distance enabled", maintainChassisDistanceEnabled);
            telemetryMap.put("maintain chassis distance correction", maintainChassisDistanceCorrection);

            telemetryMap.put("pose (x)", pose.get(0));
            telemetryMap.put("pose (y)", pose.get(1));
            telemetryMap.put("pose (heading)", Math.toDegrees(pose.get(2)));
        }

        return telemetryMap;
    }


    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public void stop() {
        for(int i = 0; i < motors.length; i++) {
            motors[i].setPower(0);
        }
    }

    //----------------------------------------------------------------------------------------------
    // Getters And Setters
    //----------------------------------------------------------------------------------------------

    public double getSwivelAngle() {
        return swivelAngle;
    }

    public double getChassisDistance() {
        return chassisDistance;
    }

    public SimpleMatrix getPose() {
        return pose;
    }

    public double getHeading(){
        return Math.toDegrees(pose.get(2));
    }

    /**
     * returns the current motor encoder positions for all three drivetrain motors:
     * [left, right, middle]
     * @return
     */
    public SimpleMatrix getWheelTicks() {
        return new SimpleMatrix(new double[][] {
                { motorFrontLeft.getCurrentPosition(), 0 },
                { motorFrontRight.getCurrentPosition(), 0 },
                { motorMiddle.getCurrentPosition(), 0 }
        });
    }

    public double getTurnRadius() {
        return targetTurnRadius;
    }

    public boolean isMaintainChassisDistanceEnabled() {
        return maintainChassisDistanceEnabled;
    }

    public void setMaintainChassisDistanceEnabled(boolean maintainChassisDistanceEnabled) {
        this.maintainChassisDistanceEnabled = maintainChassisDistanceEnabled;
    }

    public void setTargetChassisDistance(double targetChassisDistance) {
        this.targetChassisDistance = targetChassisDistance;
    }
}