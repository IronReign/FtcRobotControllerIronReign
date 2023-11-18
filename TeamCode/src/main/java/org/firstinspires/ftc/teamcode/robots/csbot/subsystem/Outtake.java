package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
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

    public static int flipperPosition = 2127;


    int slidePosition;
    public static int slidePositionMax = 2127;
    public static int slidePositionMin = 1800;

    int slideSpeed = 20;
    public static int FLIPPERINTAKEPOSITION = 2105;
    public static int FLIPPERCLEARANCEPOSITION = 1900;
    public static int FLIPPERSCOREPOSITION = 750;
    private boolean flipped = false;

    public Articulation articulate(Articulation articulation) {
        this.articulation = articulation;
        return articulation;
    }

    public enum Articulation {
        INTAKE_PIXEL,
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

    public static int intakePositionIndex = 0;
    public long intakePositionTimer = 0;
    public boolean intakePosition () {
        switch (intakePositionIndex) {
            case 0:
                intakePositionTimer = futureTime(.5);
                slide.setTargetPosition(slidePositionMax);
                if(System.nanoTime() > intakePositionTimer)
                    intakePositionIndex ++;
                break;
            case 1:
                pixelFlipper.setPosition(FLIPPERINTAKEPOSITION);
                intakePositionIndex ++;
                break;
            case 2:
                return true;
        }
        return false;
    }

    public void moveSlide(int power) {
        slidePosition += power * slideSpeed;
        if (slidePosition < slidePositionMin) {
            slidePosition = slidePositionMin;
        }
        if (slidePosition > slidePositionMax) {
            slidePosition = slidePositionMax;
        }
        slide.setTargetPosition(slidePosition);
    }






    public void raiseFlipper(int power) {
        flipperPosition += power;
    }

    public void lowerFlipper(int power) {
        flipperPosition += power;
    }


    @Override
    public void update(Canvas fieldOverlay) {
        if(articulation == Articulation.MANUAL) {
            slide.setTargetPosition(slidePosition);
            pixelFlipper.setPosition(Utils.servoNormalize(flipperPosition
                    )
            );        }
        if(articulation == Articulation.INTAKE_PIXEL) {
            if(intakePosition()) {
                articulation = Articulation.MANUAL;
            }
        }
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
        telemetryMap.put("flipper location", Utils.servoDenormalize(pixelFlipper.getPosition()));
        telemetryMap.put("flipper ticks", flipperPosition);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "OUTTAKE";
    }
}
