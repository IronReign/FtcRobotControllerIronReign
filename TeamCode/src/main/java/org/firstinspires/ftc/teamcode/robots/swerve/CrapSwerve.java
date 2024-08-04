package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Map;

@Config(value = "Swerve")
public class CrapSwerve implements Subsystem {

    HardwareMap hardwareMap;
    DcMotorEx go;
    CRServoImpl yaw;

    public DcMotorEx yawEncoder;
    int goPosition;
    public double goPower;
    public double yawPower;
    PIDController yawController;
    public int yawDegrees;
    public double ticksPerDegree = 4970.0/360;
    public double realYaw;
    public double targetYaw;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(.045, 0, 0.5);

    CrapSwerve(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        go = hardwareMap.get(DcMotorEx.class, "go");
        yaw = hardwareMap.get(CRServoImpl.class, "yaw");
        yawEncoder = hardwareMap.get(DcMotorEx.class, "encoder");
        yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        go.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        go.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        go.setMotorEnable();
        yawController = new PIDController(pidCoefficients);
        yawController.setOutputRange(-.5, .5);
        yawController.setContinuous();
        yawController.setTolerance(50);
        yawController.setInputRange(0, 360);
        yawController.enable();
    }

    @Override
    public void update(Canvas fieldOverlay) {
        yawController.setPID(pidCoefficients);
        //go.setTargetPosition(goPosition);
        go.setPower(goPower);
//        yaw.setPower(-yawPower);
        realYaw =  Utils.wrapAngle(yawEncoder.getCurrentPosition() / ticksPerDegree);
        yawController.setInput(realYaw);
        yawController.setSetpoint(targetYaw);
        yawPower = yawController.performPID() * -1;
        yaw.setPower(yawPower);
    //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    @Override
    public void stop() {
        go.setPower(0);
        yaw.setPower(0);
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
