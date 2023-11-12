package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.robots.csbot.rr_stuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.csbot.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Config(value = "CS_ROADRUNNER")
public class CSDriveTrain extends MecanumDrive implements Subsystem {
    public Robot robot;

    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;
    private final DcMotorEx rightFront;
    private final List<DcMotorEx> motors;
    private final VoltageSensor batteryVoltageSensor;

    public CSDriveTrain(HardwareMap hardwareMap, Robot robot, boolean simulated) {
        super(hardwareMap, new Pose2d(new Vector2d(0, 0), new Rotation2d(0, 0)));
        this.robot = robot;
        //TODO - implement simulations
        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            ((MotorConfigurationType) motorConfigurationType).setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse any motors using DcMotor.setDirection()

    }
    //end constructor


    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
    @Override
    public void update(Canvas fieldOverlay) {

    }
    public void drive(double x, double y, double theta) {
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -y,
                        -x
                ),
                theta
        ));

        updatePoseEstimate();
    }

    public void line() {
        Actions.runBlocking(
                actionBuilder(new Pose2d(0, 0, 0))
                        .lineToXConstantHeading(16)
                        .build());
    }
    @Override
    public void stop() {
        for (DcMotor k : motors) {
            k.setPower(0);
        }
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("x", super.pose.position.x);
        telemetryMap.put("y", super.pose.position.y);
        telemetryMap.put("heading", super.pose.heading);
        telemetryMap.put("Left Odometry Pod:\t", leftFront.getCurrentPosition());
        telemetryMap.put("Right Odometry Pod:\t", rightFront.getCurrentPosition());
        telemetryMap.put("Cross Odometry Pod:\t", rightRear.getCurrentPosition());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "DRIVETRAIN";
    }
}