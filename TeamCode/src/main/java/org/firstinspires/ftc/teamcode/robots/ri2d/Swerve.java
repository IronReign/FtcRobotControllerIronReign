package org.firstinspires.ftc.teamcode.robots.ri2d;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.HashMap;
import java.util.Map;

@Config(value = "ri2dSwerve")
public class Swerve implements Subsystem {
    HardwareMap hardwareMap;
    DcMotorEx go;
    CRServoImpl yawServo;
    public DcMotorEx yawEncoder;
    public DcMotorEx headingMotor;
    public IMU headingIMU;
    Robot robot;
    PIDController steeringController;
    public static PIDCoefficients steeringPIDCoefficients = new PIDCoefficients(.025, 0, 0.5);
    public double steeringPIDInput;
    public double realBearing;
    public double targetBearing;
    public double bearingPower;
    public double goPower;
    public double ticksPerDegree = 4920.0/360;
    public double realHeading;

    public static boolean dampenRotation = false;

    Swerve(HardwareMap hardwareMap, Robot ri2d) {
        this.hardwareMap = hardwareMap;
        go = hardwareMap.get(DcMotorEx.class, "go");
        headingMotor = hardwareMap.get(DcMotorEx.class, "turn");
        yawServo = hardwareMap.get(CRServoImpl.class, "yaw");
        yawEncoder = hardwareMap.get(DcMotorEx.class, "encoder");
        headingIMU = hardwareMap.get(IMU.class, "imu");
        headingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        headingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        headingIMU.resetYaw();
        go.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        go.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        go.setMotorEnable();
        steeringController = new PIDController(steeringPIDCoefficients);
        steeringController.setOutputRange(-.5, .5);
        steeringController.setContinuous();
        steeringController.setTolerance(50);
        steeringController.setInputRange(0, 360);
        steeringController.enable();
        robot = ri2d;
    }

    public void simplySwerve(double x, double y) {
        if(Math.hypot(x, y) > .2) {
            goPower = Math.hypot(x, y);
            targetBearing = Math.toDegrees(Math.atan2(x, y)) + 180;
        }

        else {
            targetBearing = realBearing;
            goPower = 0;
        }
        steeringController.setPID(steeringPIDCoefficients);
        steeringController.setInput(realBearing);
        realBearing =  Utils.wrapAngle(yawEncoder.getCurrentPosition() / ticksPerDegree);
        steeringPIDInput = Utils.diffBetweenAnglesDeg(targetBearing, realHeading);
        steeringController.setSetpoint(steeringPIDInput);
        bearingPower = steeringController.performPID() * -1;

        go.setPower(goPower);
        yawServo.setPower(bearingPower);
    }

    public void directDrive(boolean a, double y, double x) {
        if(a) {
            dampenRotation = !dampenRotation;
        }
        if(y > .05) {
            goPower = y;
        }
        else goPower = 0;
        if(Math.abs(x) > .05){
            bearingPower = dampenRotation ? x/4 : x;
        }
        else bearingPower = 0;

        //      go.setTargetPosition(goPosition);
        //      yaw.setPower(-yawPower);

    }

    @Override
    public void update(Canvas fieldOverlay) {
        realHeading = headingIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        drawRobot(fieldOverlay);
    }

    @Override
    public void stop() {
        go.setPower(0);
        yawServo.setPower(0);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("raw encoder position\n", yawEncoder.getCurrentPosition());
        telemetryMap.put("real bearing\t", realBearing);
        telemetryMap.put("target bearing\t", targetBearing);
        telemetryMap.put("bearing error\t", steeringController.getError());
        telemetryMap.put("bearing power\t", bearingPower);
        telemetryMap.put("goPower\t", goPower);
        telemetryMap.put("heading power\t", headingMotor.getPower());
        telemetryMap.put("heading\t", headingIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetryMap.put("steering pid input\t",steeringPIDInput);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Swerve";
    }

    public void incrementHeading(double right_stick_x) {
        headingMotor.setVelocity(right_stick_x);
    }
}
