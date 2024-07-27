package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

import java.util.Map;

public class CrapSwerve implements Subsystem {

    HardwareMap hardwareMap;
    DcMotorEx go;
    CRServoImpl yaw;

    int goPosition;
    public double goPower;
    public double yawPower;

    CrapSwerve(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        go = hardwareMap.get(DcMotorEx.class, "go");
        yaw = hardwareMap.get(CRServoImpl.class, "yaw");
        go.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        go.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        go.setMotorEnable();
    }

    @Override
    public void update(Canvas fieldOverlay) {
        //go.setTargetPosition(goPosition);
        go.setPower(goPower);
        yaw.setPower(-yawPower);

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
