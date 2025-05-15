package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Map;

@Config(value = "NSwerve")
public class NSwerve implements Subsystem {

    HardwareMap hardwareMap;
    SwerveModule swerveModule1, swerveModule2, swerveModule3;

    BNO055IMU imu;
    Orientation angles = new Orientation();
    int numSwerves;
    public double chassisHeading;
    public double lastSpeed;

    // PID parameters (adjust as needed)
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(.045, 0, 0.5);
    public static double ticksPerDegree = 4920.0 / 360;
    // Set a threshold (in degrees) below which the drive motor is allowed to run.
    public static double yawThreshold = 10;

    public NSwerve(HardwareMap hardwareMap, int numSwerves) {
        this.hardwareMap = hardwareMap;

        for(int i = 0; i < numSwerves; i++) {
            DcMotorEx driveMotor = hardwareMap.get(DcMotorEx.class, "go" + i+1);
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotor.setMotorEnable();
        }

        // Get hardware devices for the swerve module.
        DcMotorEx driveMotor1 = hardwareMap.get(DcMotorEx.class, "go1");
        CRServo yawServo1 = hardwareMap.get(CRServo.class, "yaw1");
        DcMotorEx yawEncoder1 = hardwareMap.get(DcMotorEx.class, "encoder1");

        DcMotorEx driveMotor2 = hardwareMap.get(DcMotorEx.class, "go2");
        CRServo yawServo2 = hardwareMap.get(CRServo.class, "yaw2");
        DcMotorEx yawEncoder2 = hardwareMap.get(DcMotorEx.class, "encoder2");

        DcMotorEx driveMotor3 = hardwareMap.get(DcMotorEx.class, "go3");
        CRServo yawServo3 = hardwareMap.get(CRServo.class, "yaw3");
        DcMotorEx yawEncoder3 = hardwareMap.get(DcMotorEx.class, "encoder3");

        // Reset encoders and configure the drive motor.
        yawEncoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveMotor1.setMotorEnable();

        yawEncoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveMotor2.setMotorEnable();

        yawEncoder3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        swerveModule1 = new SwerveModule(driveMotor1, yawServo1, yawEncoder1, yawPID1, ticksPerDegree, yawThreshold);
        swerveModule2 = new SwerveModule(driveMotor2, yawServo2, yawEncoder2, yawPID2, ticksPerDegree, yawThreshold);
        swerveModule3 = new SwerveModule(driveMotor3, yawServo3, yawEncoder3, yawPID3, ticksPerDegree, yawThreshold);

        initIMU();
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
            speed = drive ? deflection: 0;
        } else {
            // No new input: hold the previous target angle.
            desiredAngle = swerveModule1.getTargetAngle();
            speed = 0;
        }

        swerveModule1.setDesiredState(desiredAngle, speed);
        swerveModule2.setDesiredState(desiredAngle, speed);
        swerveModule3.setDesiredState(desiredAngle, speed);
    }
    @Override
    public void update(Canvas fieldOverlay) {
        // Update chassis heading from the IMU.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        chassisHeading = angles.firstAngle;

        // Update the swerve module control.
        swerveModule1.update();
        swerveModule2.update();
        swerveModule3.update();

        // Draw chassis/robot representation on the dashboard.
        drawRobot(fieldOverlay);
    }

    public void initIMU(){
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
}
