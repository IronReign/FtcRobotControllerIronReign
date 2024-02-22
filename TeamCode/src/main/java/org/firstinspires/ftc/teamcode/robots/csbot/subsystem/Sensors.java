package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;
import org.slf4j.helpers.Util;

@Config(value = "AA_CS_SENSORS")
public class Sensors {
    Robot robot;
    public int skyhookRightTicks;
    public int skyhookLeftTicks;

    public double rightDistSensorValue;
    public double leftDistSensorValue;
    public double skyhookIMUPitch;
    public double driveIMUYaw;


    public boolean skyhookIMUEnabled;
    public boolean distanceSensorsEnabled;
    public boolean driveIMUEnabled;

    public Sensors(Robot robot){
        this.robot = robot;
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
            driveIMUYaw = Utils.wrapAngle(robot.driveTrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        }
    }
}
