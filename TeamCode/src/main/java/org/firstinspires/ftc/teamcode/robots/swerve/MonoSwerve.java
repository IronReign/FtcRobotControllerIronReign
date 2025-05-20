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
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Utils;
import java.util.Map;

@Config(value = "Swerve")
public class MonoSwerve implements Subsystem {

    HardwareMap hardwareMap;
    SwerveModule swerveModule;

    BNO055IMU imu;
    Orientation angles = new Orientation();
    public double chassisHeading;
    public double lastSpeed;

    // PID parameters (adjust as needed)
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(.045, 0, 0.5);
    public static double ticksPerDegree = 4920.0 / 360;
    // Set a threshold (in degrees) below which the drive motor is allowed to run.
    public static double yawThreshold = 10;

    public MonoSwerve(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        // Get hardware devices for the swerve module.
        DcMotorEx driveMotor = hardwareMap.get(DcMotorEx.class, "go");
        CRServo yawServo = hardwareMap.get(CRServo.class, "yaw");
        DcMotorEx yawEncoder = hardwareMap.get(DcMotorEx.class, "encoder");

        // Reset encoders and configure the drive motor.
        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setMotorEnable();

        // Initialize the PID controller for yaw.
        PIDController yawPID = new PIDController(pidCoefficients);
        yawPID.setOutputRange(-0.5, 0.5);
        yawPID.setContinuous();
        yawPID.setTolerance(50);
        yawPID.setInputRange(0, 360);
        yawPID.enable();

        // Create the SwerveModule.
        swerveModule = new SwerveModule(driveMotor, yawServo, yawEncoder, yawPID, ticksPerDegree, yawThreshold);

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
            desiredAngle = swerveModule.getTargetAngle();
            speed = 0;
        }

        swerveModule.setDesiredState(desiredAngle, speed);
    }
    @Override
    public void update(Canvas fieldOverlay) {
        // Update chassis heading from the IMU.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        chassisHeading = angles.firstAngle;

        // Update the swerve module control.
        swerveModule.update();

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
