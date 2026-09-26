package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "BumbleBee_DriveTrain")
public class DriveTrain implements Subsystem {
    //  SET TO FALSE FOR ROBOT CENTRIC
    public static boolean fieldCentric = true;

    // MOTOR AND IMU DECLARATION
    private final DcMotorEx frontRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx rearRight;
    private final DcMotorEx rearLeft;
    private final BNO055IMU imu;

    private double headingDegrees;
    private double forward;
    private double strafe;
    private double turn;

    private double frontRightPower;
    private double frontLeftPower;
    private double rearRightPower;
    private double rearLeftPower;

    public DriveTrain(HardwareMap hardwareMap) {
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = fieldCentric ? hardwareMap.get(BNO055IMU.class, "imu") : hardwareMap.tryGet(BNO055IMU.class, "imu");

        if (imu != null) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(parameters);
        }
    }

    public void drive(double f, double s, double t) {
        forward = f;
        strafe = s;
        turn = t;
    }

    @Override
    public void readSensors() {
        if (imu != null) {
            headingDegrees = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
    }

    @Override
    public void calc(Canvas fieldOverlay) {
        double robotForward = forward;
        double robotStrafe = strafe;

        if (fieldCentric && imu != null) {
            double headingRadians = Math.toRadians(headingDegrees);
            double cosHeading = Math.cos(headingRadians);
            double sinHeading = Math.sin(headingRadians);

            robotForward = forward * cosHeading - strafe * sinHeading;
            robotStrafe = strafe * cosHeading + forward * sinHeading;
        }

        frontLeftPower = robotForward + robotStrafe + turn;
        frontRightPower = robotForward - robotStrafe - turn;
        rearLeftPower = robotForward - robotStrafe + turn;
        rearRightPower = robotForward + robotStrafe - turn;

        double maxFrontPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        double maxRearPower = Math.max(Math.abs(rearLeftPower), Math.abs(rearRightPower));
        double powerScale = Math.max(1.0, Math.max(maxFrontPower, maxRearPower));

        frontLeftPower /= powerScale;
        frontRightPower /= powerScale;
        rearLeftPower /= powerScale;
        rearRightPower /= powerScale;
    }

    @Override
    public void act() {
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        rearRight.setPower(rearRightPower);
        rearLeft.setPower(rearLeftPower);
    }

    @Override
    public void stop() {
        resetStates();
        act();
    }

    @Override
    public void resetStates() {
        forward = 0.0;
        strafe = 0.0;
        turn = 0.0;

        frontRightPower = 0.0;
        frontLeftPower = 0.0;
        rearRightPower = 0.0;
        rearLeftPower = 0.0;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetry = new LinkedHashMap<>();

        telemetry.put("Field Centric", fieldCentric && imu != null);
        telemetry.put("Heading (deg)", headingDegrees);

        return telemetry;
    }

    @Override
    public String getTelemetryName() {
        return "DriveTrain";
    }
}
