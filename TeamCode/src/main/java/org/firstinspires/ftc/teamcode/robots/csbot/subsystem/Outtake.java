package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "CS_OUTTAKE")
public class Outtake implements Subsystem {

    HardwareMap hardwareMap;
    Robot robot;

    private DcMotorEx slide = null;
    private Servo pixelFlipper = null;


    int slidePosition;
    public static int slidePositionMax = 2000;
    public static int slidePositionMin = -1000;

    int slideSpeed = 20;
    public static int FLIPPERTUCKPOSITION = 2105;
    public static int FLIPPERCLEARANCEPOSITION = 1900;
    public static int FLIPPERSCOREPOSITION = 750;
    private boolean flipped = false;

    public enum Articulation {
        MANUAL,
        SCORE_PIXEL,
        FOLD,
    }

    public enum FlipperLocation {
        TUCK,
        CLEAR,
        SCORE
    }

    FlipperLocation flipperLocation;


    //LIVE STATES
    public Articulation articulation;


    public Outtake(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;

        slide = this.hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMotorEnable();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        pixelFlipper = this.hardwareMap.get(Servo.class, "pixelFlipper");

        articulation = Articulation.MANUAL;
    }

    public void moveSlide(int power) {
        slidePosition += power * slideSpeed;
        if (slidePosition < slidePositionMin) {
            slidePosition = slidePositionMin;
        }
        if (slidePosition > slidePositionMax) {
            slidePosition = slidePositionMax;
        }
    }


    public void clearFlipper() {
        this.flipperLocation = FlipperLocation.CLEAR;
        pixelFlipper.setPosition(Utils.servoNormalize(FLIPPERCLEARANCEPOSITION));
    }

    public void tuckFlipper() {
        this.flipperLocation = FlipperLocation.TUCK;
        pixelFlipper.setPosition(Utils.servoNormalize(FLIPPERTUCKPOSITION));
    }

    public void scoreFlipper() {
        this.flipperLocation = FlipperLocation.SCORE;
        pixelFlipper.setPosition(Utils.servoNormalize(FLIPPERSCOREPOSITION));
    }

    public void flip() {
        if (flipped) {
            pixelFlipper.setPosition(Utils.servoNormalize(FLIPPERTUCKPOSITION));
            flipped = false;
        } else {
            pixelFlipper.setPosition(Utils.servoNormalize(FLIPPERSCOREPOSITION));
            flipped = true;
        }
    }

    public void raiseFlipper(int power) {
        pixelFlipper.setPosition(Utils.servoNormalize(Utils.servoDenormalize(pixelFlipper.getPosition())
                - power)
        );
    }

    public void lowerFlipper(int power) {
        pixelFlipper.setPosition(Utils.servoNormalize(Utils.servoDenormalize(pixelFlipper.getPosition())
                - power)
        );
    }


    @Override
    public void update(Canvas fieldOverlay) {
        slide.setTargetPosition(slidePosition);
    }

    @Override
    public void stop() {
        slide.setMotorDisable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("articulation", articulation.name());
        telemetryMap.put("slide position", slidePosition);
        telemetryMap.put("slide actual position", slide.getCurrentPosition());
//        telemetryMap.put("flipper position target", flipperLocation.name());
        telemetryMap.put("flipper location", Utils.servoDenormalize(pixelFlipper.getPosition())
        );
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "OUTTAKE";
    }
}
