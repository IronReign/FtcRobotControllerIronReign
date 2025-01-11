package org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.samplers;

import android.graphics.Color;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Trident;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DcMotorExResetable;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.Joint;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class Beater extends Arm {
    CRServo beater = null;


    public Beater(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        elbow = new Joint(hardwareMap, "beaterElbow", false, ELBOW_HOME_POSITION, ELBOW_PWM_PER_DEGREE, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_START_ANGLE, ELBOW_JOINT_SPEED);
        DcMotorEx bruh = this.hardwareMap.get(DcMotorEx.class, "beaterSlide");
        slide = new DcMotorExResetable(bruh);
        colorSensor = this.hardwareMap.get(NormalizedColorSensor.class, "beaterSensor");
        beater = this.hardwareMap.get(CRServo.class, "beater");
    }


    @Override
    public boolean stopOnSample() {
        colorSensorEnabled = true;
        servoPower = .8;
        if (targetSamples.contains(currentSample)) {
            servoPower = 0;
            colorSensorEnabled = false;
            return true;
        }
        return false;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        beater.setPower(-servoPower);
        slide.setTargetPosition(slideTargetPosition);
        if (colorSensorEnabled) {
            updateColorSensor();
        }
    }

    @Override
    public void stop() {
        servoPower = 0;
        beater.setPower(0);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("slide target : real", slideTargetPosition + " : " + slide.getCurrentPosition());
        telemetryMap.put("current sample", currentSample.name());
        telemetryMap.put("colorsensor", colorSensorEnabled);
        if (colorSensorEnabled) {
            telemetryMap.put("colorsensor hsv", "" + HSVasString());
            telemetryMap.put("colorsensor rgb", colorSensor.getNormalizedColors().red + " " + colorSensor.getNormalizedColors().green + " " + colorSensor.getNormalizedColors().blue);
        }
        telemetryMap.put("beater speed", servoPower);
        return telemetryMap;
    }


    @Override
    public boolean intake() {

        return false;
    }

    @Override
    public boolean outtake() {
        return false;
    }

    @Override
    boolean tuck() {
        return false;
    }

    @Override
    public String updateColorSensor() {
        double hue = getHSV()[0];
        if (hue < 45 && hue > 35) {
            currentSample = Sample.NEUTRAL;
            return "NEUTRAL";
        } else if (hue < 15 || hue > 350) {
            currentSample = Sample.RED;
            return "RED";
        } else if (hue < 225 && hue > 200) {
            currentSample = Sample.BLUE;
            return "BLUE";
        } else {
            currentSample = Sample.NO_SAMPLE;
            return "NO SAMPLE";
        }
    }


    @Override
    public String getTelemetryName() {
        return "Beater";
    }
}
