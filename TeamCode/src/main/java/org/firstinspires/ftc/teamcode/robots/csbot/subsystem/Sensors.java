package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
import org.slf4j.helpers.Util;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "AA_CS_SENSORS")
public class Sensors implements Subsystem {
    Robot robot;
    public int skyhookRightTicks;
    public int skyhookLeftTicks;

    public double rightDistSensorValue;
    public double leftDistSensorValue;
    public double skyhookIMUPitch;
    public double driveIMUYaw;
    public double beaterBarAmps;
    public int outtakeSlideTicks;


    public static boolean skyhookIMUEnabled;
    public static boolean distanceSensorsEnabled;
    public static boolean driveIMUEnabled;

    public Sensors(Robot robot){
        this.robot = robot;
        skyhookIMUEnabled = false;
        driveIMUEnabled = true;
        distanceSensorsEnabled = false;
    }

    public void update(){
        //SKYHOOK SENSORS
        skyhookLeftTicks = robot.skyhook.skyhookLeft.getCurrentPosition();
        skyhookRightTicks = robot.skyhook.skyhookRight.getCurrentPosition();
        if(skyhookIMUEnabled)
            skyhookIMUPitch = Utils.wrapAngle(robot.skyhook.skyhookIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));

        //DRIVETRAIN
        if(distanceSensorsEnabled) {
            rightDistSensorValue = robot.driveTrain.rightDistanceSensor.getDistance(DistanceUnit.INCH);
            leftDistSensorValue = robot.driveTrain.leftDistanceSensor.getDistance(DistanceUnit.INCH);
        }
        if(driveIMUEnabled){
            driveIMUYaw = robot.driveTrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + (alliance.getMod()? -90 : 90);
        }

        //INTAKE
        beaterBarAmps = robot.intake.beater.getCurrent(CurrentUnit.AMPS);

        //OUTTAKE
        outtakeSlideTicks = robot.outtake.slide.getCurrentPosition();

    }

    @Override
    public void update(Canvas fieldOverlay) {
        update();
    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return new LinkedHashMap<>();
    }

    @Override
    public String getTelemetryName() {
        return null;
    }
}
