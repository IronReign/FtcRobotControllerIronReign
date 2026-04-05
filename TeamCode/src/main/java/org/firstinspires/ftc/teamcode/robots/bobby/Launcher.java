
package org.firstinspires.ftc.teamcode.robots.bobby;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Subsystem;


import java.util.LinkedHashMap;
import java.util.Map;


public class Launcher implements Subsystem {
    private static final int START_SERVO_POSITION = 1390;
    private static final int PUSH_UP = 1070;
    private static final double FLYWHEEL_TICKS_PER_REV = 1120.0;


    private final DcMotorEx flywheelL;
    private final DcMotorEx flywheelR;
    private final Servo flywheelPusher;


    public Launcher(HardwareMap hardwareMap) {
        flywheelL = hardwareMap.get(DcMotorEx.class, "flywheelL");
        flywheelR = hardwareMap.get(DcMotorEx.class, "flywheelR");
        flywheelPusher = hardwareMap.get(Servo.class, "flywheelPusher");


        flywheelL.setDirection(DcMotor.Direction.REVERSE);


        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flywheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flywheelL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30, 0, 2, 11));
        flywheelR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(30, 0, 2, 11));


        setPusherDown();
    }


    public void setFlywheelVelocity(double rpm) {
        double ticksPerSecond = (rpm * FLYWHEEL_TICKS_PER_REV) / 60.0;
        flywheelL.setVelocity(ticksPerSecond);
        flywheelR.setVelocity(ticksPerSecond);
    }


    public void setPusherUp() {
        flywheelPusher.setPosition(servoNormalize(PUSH_UP));
    }


    public void setPusherDown() {
        flywheelPusher.setPosition(servoNormalize(START_SERVO_POSITION));
    }


    public double getFlywheelLeftRpm() {
        return flywheelL.getVelocity() * 60 / FLYWHEEL_TICKS_PER_REV;
    }


    public double getFlywheelRightRpm() {
        return flywheelR.getVelocity() * 60 / FLYWHEEL_TICKS_PER_REV;
    }


    @Override
    public void update(Canvas fieldOverlay) {
    }


    @Override
    public void stop() {
        setFlywheelVelocity(0.0);
        setPusherDown();
    }


    @Override
    public void resetStates() {
        stop();
    }


    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String, Object> telemetry = new LinkedHashMap<>();
        telemetry.put("Flywheel L velocity", getFlywheelLeftRpm());
        telemetry.put("Flywheel R velocity", getFlywheelRightRpm());
        return telemetry;
    }


    @Override
    public String getTelemetryName() {
        return "Shooter";
    }


    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0;
    }
}