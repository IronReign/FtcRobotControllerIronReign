package org.firstinspires.ftc.teamcode.robots.ri2d;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;

import java.util.Map;

public class Outtake implements Subsystem {
    HardwareMap hardwareMap;
    DcMotorEx linearSlide;
    Robot robot;
    public Outtake(HardwareMap hardwareMap, Robot ri2d){
        this.hardwareMap = hardwareMap;
        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot = ri2d;
    }
    @Override
    public void update(Canvas fieldOverlay) {

    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return null;
    }
}
