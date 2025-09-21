package org.firstinspires.ftc.teamcode.robots.swerve;

import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.notJoystickDeadZone;
import static org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils.wrapAngle;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.old.Sensors;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;
import org.firstinspires.ftc.teamcode.util.utilMethods;

import java.util.Map;

@Config(value = "swerve")
public class DualSwerve implements Subsystem {

    HardwareMap hardwareMap;
    SwerveModule swerveModuleOne;
    SwerveModule swerveModuleTwo;
    BNO055IMU imu;
    Orientation angles = new Orientation();
    public double chassisHeading;
    public double lastSpeed;

    // PID parameters (adjust as needed)
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(.045, 0, 0.5);
    public static double ticksPerDegree = 4920.0 / 360;
    // Set a threshold (in degrees) below which the drive motor is allowed to run.
    public static double yawThreshold = 10;

    public static PIDCoefficients turnCoefficients = new PIDCoefficients(.1, 0, 0);
    public PIDController turnController = new PIDController(pidCoefficients);
    public static double PIDCorrection = 0;
    public static double PIDError = 0;

    public static double desiredChassisAngle;

    public static boolean driving = false;
    public static boolean enabled = true;

    public DualSwerve(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        // Get hardware devices for the swerve module.
        DcMotorEx driveMotorOne = hardwareMap.get(DcMotorEx.class, "goOne");
        CRServo yawServoOne = hardwareMap.get(CRServo.class, "yawOne");
        DcMotorEx yawEncoderOne = hardwareMap.get(DcMotorEx.class, "encoderOne");
        AnalogInput a0 = hardwareMap.get(AnalogInput.class, "a0");

        DcMotorEx driveMotorTwo = hardwareMap.get(DcMotorEx.class, "goTwo");
        CRServo yawServoTwo = hardwareMap.get(CRServo.class, "yawTwo");
        DcMotorEx yawEncoderTwo = hardwareMap.get(DcMotorEx.class, "encoderTwo");
        AnalogInput a1 = hardwareMap.get(AnalogInput.class, "a1");

        // Reset encoders and configure the drive motor.
        yawEncoderOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotorOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotorOne.setMotorEnable();

        yawEncoderTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotorTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotorTwo.setMotorEnable();

        // Initialize the PID controller for yaw.
        PIDController yawPIDOne = new PIDController(pidCoefficients);
        yawPIDOne.setOutputRange(-0.5, 0.5);
        yawPIDOne.setContinuous();
        yawPIDOne.setTolerance(50);
        yawPIDOne.setInputRange(0, 360);
        yawPIDOne.enable();

        PIDController yawPIDTwo = new PIDController(pidCoefficients);
        yawPIDTwo.setOutputRange(-0.5, 0.5);
        yawPIDTwo.setContinuous();
        yawPIDTwo.setTolerance(50);
        yawPIDTwo.setInputRange(0, 360);
        yawPIDTwo.enable();

        turnController.setOutputRange(-0.5, 0.5);
        turnController.setContinuous();
        turnController.setTolerance(50);
        turnController.setInputRange(0, 360);
        turnController.enable();


        // Create the SwerveModule.
        swerveModuleOne = new SwerveModule(driveMotorOne, yawServoOne, yawEncoderOne, a0, yawPIDOne, ticksPerDegree, yawThreshold);
        swerveModuleTwo = new SwerveModule(driveMotorTwo, yawServoTwo, yawEncoderTwo, a1, yawPIDTwo, ticksPerDegree, yawThreshold);

        initIMU();
    }

    /**
     * This method converts driver inputs (from the left joystick) into a desired velocity:
     * - The direction (theta) is calculated from the x and y deflections.
     * - The speed is set from the overall deflection.
     * The chassis IMU heading is added so that the command is field oriented.
     *
     * @param leftJoystickX left stick X (lateral)
     * @param leftJoystickY left stick Y (forward/backward)
     */
    public void processDriverInputDrive(double leftJoystickX, double leftJoystickY, boolean drive) {
        double leftDeflection = Math.hypot(leftJoystickX, leftJoystickY);
        double desiredAngleOne;
        double speedOne;
        double desiredAngleTwo;
        double speedTwo;

        if (leftDeflection > 0.2) {  // Only update desired angle if joystick deflection is significant.
            driving = true;
            // Compute the desired angle from joystick input and add chassisHeading for field orientation.
            desiredAngleOne = Utils.wrapAngle(Math.toDegrees(Math.atan2(leftJoystickX, leftJoystickY)) + chassisHeading);
            speedOne = drive ? leftDeflection : 0;
            desiredAngleTwo = Utils.wrapAngle(Math.toDegrees(Math.atan2(leftJoystickX, leftJoystickY)) + chassisHeading);
            speedTwo = drive ? leftDeflection : 0;
        } else {
            driving = false;
            // No new input: hold the previous target angle.
            desiredAngleOne = swerveModuleOne.getTargetAngle();
            speedOne = 0;
            desiredAngleTwo = swerveModuleTwo.getTargetAngle();
            speedTwo = 0;
        }

        swerveModuleOne.setDesiredState(desiredAngleOne, speedOne);
        swerveModuleTwo.setDesiredState(desiredAngleTwo, speedTwo);

        if (swerveModuleOne.swerveAligned && swerveModuleTwo.swerveAligned) {
            swerveModuleTwo.aligned(speedOne);
            swerveModuleOne.aligned(speedTwo);
        }
    }


    public void processDriverInputRotation(double rightJoyStickX, double rightJoyStickY, boolean power) {
        double rightDeflection = Math.hypot(rightJoyStickX, rightJoyStickY);


        if (rightDeflection > 0.2) {
            desiredChassisAngle = Utils.wrapAngle(Math.toDegrees(Math.atan2(rightJoyStickX, rightJoyStickY)));

//            angleDiff = utilMethods.angleDifference(desiredChassisAngle, chassisHeading);
            //target = desiredChassisAngle
            //pid loop that corrects the chassisHeading to the target

            //driving one wheel forward, and the other back, will allow us to turn


//            speed = power ? rightDeflection : 0;
//            if(desiredChassisAngle > 0 && desiredChassisAngle <= 179){
//                swerveModuleOne.setDesiredState(0, speed, true, false);
//                swerveModuleTwo.setDesiredState(0, speed, true, true);
//
//            }
//            else if( desiredChassisAngle >= 180 && desiredChassisAngle <= 360){
//                swerveModuleOne.setDesiredState(0, speed, true, true);
//                swerveModuleTwo.setDesiredState(0, speed, true, false);
//            }
//            else{
//                swerveModuleOne.setDesiredState(0, speed, false, false);
//                swerveModuleTwo.setDesiredState(0, speed, false, false);
//            }
        }
    }

    @Override
    public void update(Canvas fieldOverlay) {
        // Update chassis heading from the IMU.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        chassisHeading = angles.firstAngle;

        if (enabled) {
            if (!driving)
                holdHeading();
            // Update the swerve module control.
            swerveModuleOne.update();
            swerveModuleTwo.update();
        }
        // Draw chassis/robot representation on the dashboard.
        drawRobot(fieldOverlay);
    }

    public boolean holdHeading() {
        turnController.setInput(chassisHeading + 180);
        turnController.setSetpoint(desiredChassisAngle);
        turnController.setOutputRange(-.8, .8);
        turnController.setTolerance(3);
        double correction = turnController.performPID();
        PIDCorrection = correction;
        PIDError = turnController.getError();
        if (turnController.onTarget()) {

            return true;
        } else {
            swerveModuleOne.setDesiredState(0, correction);
            swerveModuleTwo.setDesiredState(0, correction);
            turnController.enable();

            return false;
        }

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
        return "Swerve";
    }

    public void resetEncoders() {
        swerveModuleOne.resetEncoders();
        swerveModuleTwo.resetEncoders();
    }
}
