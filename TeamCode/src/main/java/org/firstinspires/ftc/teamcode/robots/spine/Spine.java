package org.firstinspires.ftc.teamcode.robots.spine;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

import java.util.Map;

@Config(value = "Spine")
public class Spine implements Subsystem {

    HardwareMap hardwareMap;

    DcMotorEx backLeft;
    DcMotorEx backRight;
    Servo muscle;
    double driveDamp = .5;
    double turnDamp = .3;
    double muscleTicks = 1500;
    double rightPower = 0, leftPower = 0;
    public Spine(HardwareMap hardwareMap) {
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        muscle = hardwareMap.get(Servo.class, "muscle");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.hardwareMap = hardwareMap;
    }

    public void directDrive(double lt, double rt, double y, double x) {
        if(Math.abs(y) > .05) {
             rightPower = y;
             leftPower = y;

             rightPower *= driveDamp;
             leftPower *= driveDamp;
        }
        else if(Math.abs(x) > .05) {
            rightPower = x;
            leftPower = -x;

            rightPower *= turnDamp;
            leftPower *= turnDamp;
        }
        else {
            rightPower = 0;
            leftPower = 0;
        }
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);

        if(Math.abs(lt) >= .3) {
            if(muscleTicks < 2240)
                muscleTicks += 2;
        }
        if (Math.abs(rt) >= .3){
            if(muscleTicks > 510)
                muscleTicks -= 2;
        }
        muscle.setPosition(Utils.servoNormalize(muscleTicks));
    }

    @Override
    public void update(Canvas fieldOverlay) {
        drawRobot(fieldOverlay);
    }

    public void drawRobot(Canvas c) {
        final double ROBOT_RADIUS = 5;

//        c.setStrokeWidth(1);
//        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);
//
//        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
//        Vector2d p1 = t.position.plus(halfv);
//        Vector2d p2 = p1.plus(halfv);
//        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    @Override
    public void stop() {
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {return "Spine";}
}
