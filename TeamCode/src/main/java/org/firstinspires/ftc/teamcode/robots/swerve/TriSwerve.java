package org.firstinspires.ftc.teamcode.robots.swerve;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;
import static org.firstinspires.ftc.teamcode.util.utilMethods.withinError;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.swervolicious.subsystem.SwerveModule;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;
import org.firstinspires.ftc.teamcode.robots.swervolicious.rr_localize.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.rrQuickStart.Localizer;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Map;

@Config(value = "TriSwerve")
public class TriSwerve implements Subsystem {
    public static int quickTripDistance = 215;
    public static int fourCornersDistance = 40;
    public static double IN_PER_TICK = 0.00293063133;

    HardwareMap hardwareMap;
    Limelight3A limelight;
    SwerveModule swerveModule1, swerveModule2, swerveModule3;
    public SwerveModule[] modules;
    Servo gripper;
    Localizer localizer;

    BNO055IMU imu;
    Orientation angles = new Orientation();
    public double chassisHeading;
    public double lastSpeed;

    // PID parameters (adjust as needed)
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(.045, 0, 0.5);
    public static double ticksPerDegree = 4920.0 / 360;
    // Set a threshold (in degrees) below which the drive motor is allowed to run.
    public static double yawThreshold = 10;
    public static boolean gripperOpen = true;
    public static int gripperOpenPosition = 1500;
    public static int gripperClosedPosition = 1100;
    public static int gripperStartPosition = 899;
    public boolean gripperReady = false;

    public TriSwerve(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        localizer = new PinpointLocalizer(hardwareMap, IN_PER_TICK, new Pose2d(new Vector2d(0, 0), 0));
        // Get hardware devices for the swerve modules.
        DcMotorEx driveMotor1 = hardwareMap.get(DcMotorEx.class, "go0");
        CRServo yawServo1 = hardwareMap.get(CRServo.class, "yaw0");
        DcMotorEx yawEncoder1 = hardwareMap.get(DcMotorEx.class, "encoder0");
        AnalogInput a0 = hardwareMap.get(AnalogInput.class, "a0");

        DcMotorEx driveMotor2 = hardwareMap.get(DcMotorEx.class, "go1");
        CRServo yawServo2 = hardwareMap.get(CRServo.class, "yaw1");
        DcMotorEx yawEncoder2 = hardwareMap.get(DcMotorEx.class, "encoder1");
        AnalogInput a1 = hardwareMap.get(AnalogInput.class, "a1");

        DcMotorEx driveMotor3 = hardwareMap.get(DcMotorEx.class, "go2");
        CRServo yawServo3 = hardwareMap.get(CRServo.class, "yaw2");
        DcMotorEx yawEncoder3 = hardwareMap.get(DcMotorEx.class, "encoder2");
        AnalogInput a2 = hardwareMap.get(AnalogInput.class, "a2");

        gripper = hardwareMap.get(Servo.class, "gripper");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Reset encoders and configure the drive motor.
        yawEncoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor1.setMotorEnable();

        yawEncoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor2.setMotorEnable();

        yawEncoder3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor3.setMotorEnable();

        // Initialize the PID controller for yaw.
        PIDController yawPID1 = new PIDController(pidCoefficients);
        yawPID1.setOutputRange(-0.5, 0.5);
        yawPID1.setContinuous();
        yawPID1.setTolerance(50);
        yawPID1.setInputRange(0, 360);
        yawPID1.enable();

        PIDController yawPID2 = new PIDController(pidCoefficients);
        yawPID2.setOutputRange(-0.5, 0.5);
        yawPID2.setContinuous();
        yawPID2.setTolerance(50);
        yawPID2.setInputRange(0, 360);
        yawPID2.enable();

        PIDController yawPID3 = new PIDController(pidCoefficients);
        yawPID3.setOutputRange(-0.5, 0.5);
        yawPID3.setContinuous();
        yawPID3.setTolerance(50);
        yawPID3.setInputRange(0, 360);
        yawPID3.enable();


        // Create the SwerveModule.
        swerveModule1 = new SwerveModule("back",driveMotor1, yawServo1, yawEncoder1, a0,yawPID1, ticksPerDegree, yawThreshold);
        swerveModule2 = new SwerveModule("right",driveMotor2, yawServo2, yawEncoder2, a1 ,yawPID2, ticksPerDegree, yawThreshold);
        swerveModule3 = new SwerveModule("left",driveMotor3, yawServo3, yawEncoder3, a2,yawPID3, ticksPerDegree, yawThreshold);
        modules = new SwerveModule[] {swerveModule1, swerveModule2, swerveModule3};
        initIMU();
    }

    public void rotate(double turnPower) {

        // Where each module sits, measured CCW from the robot’s +Y (forward) axis
        // -- adjust these three numbers if your frame layout is different.
        final double[] MODULE_BEARINGS = {0, 240, 120};   // deg

        // Tangential direction = bearing ±90 deg
        double tangentOffset = turnPower >= 0 ? 90 : -90;
        double speed = Math.abs(turnPower);                 // 0-1 drive power

        // Module 1
        double tgt1 = Utils.wrapAngle(MODULE_BEARINGS[0] + tangentOffset);
        swerveModule1.setDesiredState(tgt1, speed);

        // Module 2
        double tgt2 = Utils.wrapAngle(MODULE_BEARINGS[1] + tangentOffset);
        swerveModule2.setDesiredState(tgt2, speed);

        // Module 3
        double tgt3 = Utils.wrapAngle(MODULE_BEARINGS[2] + tangentOffset);
        swerveModule3.setDesiredState(tgt3, speed);
    }

    /**
     * This method converts driver inputs (from the left joystick) into a desired velocity:
     * - The direction (theta) is calculated from the x and y deflections.
     * - The speed is set from the overall deflection.
     * The chassis IMU heading is added so that the command is field oriented.
     *
     * @param joystickX left stick X (lateral)
     * @param joystickY left stick Y (forward/backward)
     */
    public void processDriverInput(double joystickX, double joystickY, boolean drive) {
        double deflection = Math.hypot(joystickX, joystickY);
        double desiredAngle;
        double speed;

        if (deflection > 0.2) {  // Only update desired angle if joystick deflection is significant.
            // Compute the desired angle from joystick input and add chassisHeading for field orientation.
            desiredAngle = Utils.wrapAngle(Math.toDegrees(Math.atan2(joystickX, joystickY)) + chassisHeading);
            speed = drive ? deflection : 0;
        } else {
            // No new input: hold the previous target angle.
            desiredAngle = swerveModule1.getTargetAngle();
            speed = 0;
        }

        swerveModule1.setDesiredState(desiredAngle, speed);
        swerveModule2.setDesiredState(desiredAngle, speed);
        swerveModule3.setDesiredState(desiredAngle, speed);
    }

    public int quickTripStage = 0;
    public long quickTripTimer = 0;

    public boolean quickTrip() {
        switch (quickTripStage) {
            case 0:
                processDriverInput(0, 1, true);
                if (localizer.getPose().position.x > quickTripDistance) {
                    processDriverInput((localizer.getPose().position.y / Math.abs(localizer.getPose().position.y)) * 1, 0, false);
                    quickTripStage++;
                    quickTripTimer = futureTime(.5);
                }
                break;

            case 1:
                processDriverInput((localizer.getPose().position.y / Math.abs(localizer.getPose().position.y)) * 1, 0, false);
                if (isPast(quickTripTimer)) {
                    quickTripStage++;
                }

                break;
            case 2:
                processDriverInput((localizer.getPose().position.y / Math.abs(localizer.getPose().position.y)) * 1, 0, true);

                if (withinError(localizer.getPose().position.y, 0, 5)) {
                    processDriverInput(0, 0, false);
                    quickTripTimer = futureTime(2);
                    processDriverInput(0, -1, false);
                    quickTripStage++;
                }
                break;
            case 3:
                processDriverInput(0, -1, false);
                if (isPast(quickTripTimer)) {
                    quickTripStage++;
                }

                break;

            case 4:
                processDriverInput(0, -1, true);

                if (localizer.getPose().position.x < 0) {
                    processDriverInput((localizer.getPose().position.y / Math.abs(localizer.getPose().position.y)) * 1, 0, false);
                    quickTripStage++;
                    quickTripTimer = futureTime(1);
                }
                break;
            case 5:
                processDriverInput((localizer.getPose().position.y / Math.abs(localizer.getPose().position.y)) * 1, 0, true);
                if (withinError(localizer.getPose().position.y, 0, 5)) {
                    processDriverInput(0, 0, false);
                    return true;
                }
                break;
        }
        return false;
    }

    int fourCornersStage = 0;

    public boolean fourCorners() {
        switch (fourCornersStage) {
            case 1:

                break;
            case 2:
                break;
            case 3:
                break;
        }

        return false;
    }

    public double getCanTx() {
        limelight.pipelineSwitch(6);
        LLResult llResult;
//                break;
//            case 1:
        if ((llResult = limelight.getLatestResult()) != null) {
            return llResult.getTx();
        }
        else return -256;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        // Update chassis heading from the IMU.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        chassisHeading = angles.firstAngle;

        // Update the swerve module control.
        swerveModule1.update(fieldOverlay);
        swerveModule2.update(fieldOverlay);
        swerveModule3.update(fieldOverlay);
        localizer.update();
        if(gripperReady) {
            gripper.setPosition(Utils.servoNormalize(gripperOpen ? gripperOpenPosition : gripperClosedPosition));
        }
        else
            gripper.setPosition(gripperStartPosition);

        // Draw chassis/robot representation on the dashboard.
        drawRobot(fieldOverlay);
    }

    boolean calibrationStarted = false;
    @Override
    public boolean calibrate() {

        if (!calibrationStarted) {
            calibrationStarted = true;
            swerveModule1.startCalibration();
            swerveModule2.startCalibration();
            swerveModule3.startCalibration();
        }
//        if(swerveModule1.calibrate() && swerveModule2.calibrate() && swerveModule3.calibrate())
        return true;
                //(swerveModule1.calibrate()) &&
                    //(swerveModule2.calibrate()) &&
                    //(swerveModule3.calibrate());
//        return false;
    }

    public void initIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    public void drawRobot(Canvas c) {
        final double ROBOT_RADIUS = 5;
        // (Drawing code can be implemented here if desired.)
    }

    @Override
    public void stop() {
        // Stop both the drive motor and yaw servo.
        // You may also add logic here to reset module state.
        // For example:
        // swerveModule.setDesiredState(swerveModule.getCurrentAngle(), 0);
    }

    @Override
    public void resetStates() {
        // Implement if needed.
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return "TriSwerveClassic";
    }
}
