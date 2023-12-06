package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

public class Skyhook implements Subsystem {
    HardwareMap hardwareMap;
    Robot robot;
    public static int skyhookTicks = 0;
    public static int jimmyTicks = 1500;
    public static int JIMMY_TENSION_TICKS = 1500;
    public static int JIMMY_RELEASE_TICKS = 2100;
    DcMotor kareem, jabbar;
    Servo jimmy;

    public Skyhook(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        initMotors();
        jimmy = hardwareMap.get(Servo.class, "jimmy");
        jimmyTicks = JIMMY_TENSION_TICKS;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        jimmy.setPosition(Utils.servoNormalize(jimmyTicks));
        jabbar.setTargetPosition(skyhookTicks);
        kareem.setTargetPosition(skyhookTicks);
    }

    public void releaseTheJimmy() {
        jimmyTicks = JIMMY_RELEASE_TICKS;
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        telemetryMap.put("skyhookTicks", skyhookTicks);
        telemetryMap.put("jimmyTicks", jimmyTicks);

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "SKYHOOK";
    }

    public void initMotors() {
        kareem = this.hardwareMap.get(DcMotorEx.class, "kareem");
        jabbar = this.hardwareMap.get(DcMotorEx.class, "jabbar");
        kareem.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kareem.setTargetPosition(0);
        kareem.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        jabbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jabbar.setTargetPosition(0);
        jabbar.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        kareem.setPower(1);
        jabbar.setPower(1);
    }
}
