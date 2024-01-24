package org.firstinspires.ftc.teamcode.robots.goldenduck.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.LinkedHashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

@Config(value = "GD_SKYHOOK")
public class Skyhook implements Subsystem {

    HardwareMap hardwareMap;
    Robot robot;

    private DcMotorEx skyHookRight, skyHookLeft = null;

    public void setSkyHookTargetPosition(int skyHookTargetPosition) {
        if (skyHookTargetPosition < skyHookPositionMin) {
            skyHookTargetPosition = skyHookPositionMin;
        }
        if (skyHookTargetPosition > skyHookPositionMax) {
            skyHookTargetPosition = skyHookPositionMax;
        }
        this.skyHookTargetPosition = skyHookTargetPosition;
    }

    public static int skyHookTargetPosition = 0; //make non public
    public static int skyHookPositionMax = 1000; //todo find real skyhook Max
    public static int skyHookPositionMin = 0;

    //todo, find these values:
    public static int skyHookPrep = 900; //extension needed to get hooks above the rigging
    public static int skyHookLift = 500; //position needed to pull robot off of the mat

    int skyhookSpeed = 20;

    public Skyhook(HardwareMap hardwareMap, Robot robot) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;

        skyHookRight = this.hardwareMap.get(DcMotorEx.class, "motorSkyHookRight");
        skyHookRight.setMotorEnable();
        skyHookRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        skyHookRight.setTargetPosition(0);
        skyHookRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        skyHookRight.setPower(1);

        skyHookLeft = this.hardwareMap.get(DcMotorEx.class, "motorSkyHookLeft");
        skyHookLeft.setMotorEnable();
        //skyHookLeft.setDirection(DcMotor.Direction.REVERSE); //todo is this the correct motor to reverse?
        skyHookLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        skyHookLeft.setTargetPosition(0);
        skyHookLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        skyHookLeft.setPower(1);

    }

    public void adjustSkyHooks(double gamepadSpeed) {
        setSkyHookTargetPosition(skyHookRight.getCurrentPosition()+(int)(gamepadSpeed * skyhookSpeed));
    }

    public int getSkyHookTargetPosition() {
        return skyHookTargetPosition;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        skyHookLeft.setTargetPosition(skyHookTargetPosition);
        skyHookRight.setTargetPosition(skyHookTargetPosition);
    }

    @Override
    public void stop() {

        skyHookRight.setMotorDisable();
        skyHookLeft.setMotorDisable();
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("skyhook target position", skyHookTargetPosition);
        telemetryMap.put("skyhook actual position", skyHookRight.getCurrentPosition());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "SKYHOOK";
    }
}
