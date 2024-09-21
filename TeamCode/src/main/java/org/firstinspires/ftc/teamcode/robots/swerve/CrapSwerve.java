package org.firstinspires.ftc.teamcode.robots.swerve;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.Map;

@Config(value = "Swerve")
public class CrapSwerve implements Subsystem {

    HardwareMap hardwareMap;
    DcMotorEx go;
    CRServoImpl yaw;

    public DcMotorEx yawEncoder;
    public static boolean dampenRotation = false;
    public double goPower;
    public double yawPower;
    PIDController yawController;
    public double ticksPerDegree = 4920.0/360;
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

    public void simplySwerve(double x, double y, double power) {
        goPower = power;
        if(Math.hypot(x, y) > .2) {
            targetYaw = Math.toDegrees(Math.atan2(x, y)) + 180;
        }
        else {
            targetYaw = realYaw;
            goPower = 0;
        }

        yawController.setPID(pidCoefficients);
        go.setPower(goPower);
        realYaw =  Utils.wrapAngle(yawEncoder.getCurrentPosition() / ticksPerDegree);
        yawController.setInput(realYaw);
        yawController.setSetpoint(targetYaw);
        yawPower = yawController.performPID() * -1;
        yaw.setPower(yawPower);
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
            yawPower = dampenRotation ? x/4 : x;
        }
        else yawPower = 0;

        //      go.setTargetPosition(goPosition);
        //      yaw.setPower(-yawPower);

    }

    @Override
    public void update(Canvas fieldOverlay) {
        drawRobot(fieldOverlay);
    }

    @Override
    public void stop() {
        go.setPower(0);
        yaw.setPower(0);
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
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return "Swerve";
    }
}
