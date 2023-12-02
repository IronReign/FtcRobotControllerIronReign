package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.csbot.util.Joint;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "CS_OUTTAKE")
public class Outtake implements Subsystem {

    HardwareMap hardwareMap;
    Robot robot;

    private DcMotorEx slide = null;
    private Servo pixelFlipper = null;
    public Joint flipper;

    public static int flipperPosition = 1888;


    int slidePosition = 0;
    public static int slidePositionMax = 1600;
    public static int slidePositionMin = 0;

    int slideSpeed = 20;
    public static int UNTUCK_SLIDE_POSITION = 0;
    public static int FLIPPERINTAKEPOSITION = 1888;
    public static int FLIPPER_INIT_POSITION = 1439;

    public static boolean TEMP_FLIPPER_TUNE = false;

    //FLIPPER JOINT VARIABLES
    public static int FLIPPER_HOME_POSITION = 1888;
    public static double FLIPPER_PWM_PER_DEGREE = 7.35;
    public static double FLIPPER_START_ANGLE = 0.1;
    //IN DEGREES PER SECOND
    public static double FLIPPER_JOINT_SPEED = 2;

    public static double FLIPPER_MIN_ANGLE = 1;
    public static double FLIPPER_MAX_ANGLE = 62.8571428571;
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
        flipper = new Joint(hardwareMap, "pixelFlipper", false, FLIPPER_HOME_POSITION, FLIPPER_PWM_PER_DEGREE, FLIPPER_MIN_ANGLE, FLIPPER_MAX_ANGLE, FLIPPER_START_ANGLE, FLIPPER_JOINT_SPEED);
        slide = this.hardwareMap.get(DcMotorEx.class, "slide");
        slide.setMotorEnable();
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
//        pixelFlipper = this.hardwareMap.get(Servo.class, "pixelFlipper");

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


public void flipperTest(){
    if(TEMP_FLIPPER_TUNE) {
        flipper.setTargetAngle(flipper.getCurrentAngle() + 1);
    }
    else
        flipper.setTargetAngle(flipper.getCurrentAngle() - 1);
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
        flipper.update();
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
        telemetryMap.put("flipper angle", flipper.getCurrentAngle());
        telemetryMap.put("flipper target angle", flipper.getTargetAngle());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "OUTTAKE";
    }
}
