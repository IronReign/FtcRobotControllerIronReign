package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import static org.firstinspires.ftc.teamcode.robots.deepthought.IntoTheDeep_6832.alliance;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "CS_SENSORS")
public class Sensors implements Subsystem {
    Robot robot;
    public int skyhookRightTicks;
    public int skyhookLeftTicks;

    public boolean leftTouchSensor = false;
    public boolean rightTouchSensor = false;
    public static boolean touchSensorsEnabled = false;

    public double rightDistSensorValue;
    public double leftDistSensorValue;
    public double averageDistSensorValue;
    public double skyhookIMUPitch;
    public double driveIMUYaw;
    public double beaterBarAmps;
    public int outtakeSlideTicks;
    public double rightPixelSensorValue, leftPixelSensorValue;

    public static boolean skyhookIMUEnabled;
    public static boolean distanceSensorsEnabled;
    public static boolean driveIMUEnabled;
    public static boolean pixelSensorEnabled;

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
            averageDistSensorValue = (leftDistSensorValue + rightDistSensorValue)/2;
        }
        else {
            rightDistSensorValue = 500;
            leftDistSensorValue = 500;
            averageDistSensorValue = (leftDistSensorValue + rightDistSensorValue)/2;
        }
        if(driveIMUEnabled){ //todo - is this test valid? don't we always want a heading? also, need to validate if this is a legit way to get heading once we start swapping out pinpoint, sparkfun optical, etc.
            //driveIMUYaw = Utils.wrapAngle(robot.driveTrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + (alliance.isRed()? -90 : 90) + (CenterStage_6832.frontAuton? 180: 0));
            driveIMUYaw = Utils.wrapAngle(Math.toDegrees(robot.driveTrain.pose.heading.log()) + (alliance.isRed()? -90 : 90));
        }

        if(touchSensorsEnabled){
            leftTouchSensor = robot.skyhook.leftTouchSensor.isPressed();
            rightTouchSensor = robot.skyhook.rightTouchSensor.isPressed();
        }

//        if(pixelSensorEnabled) {
//            rightPixelSensorValue = robot.intake.pixelSensorRight.getDistance(DistanceUnit.INCH);
//            leftPixelSensorValue = robot.intake.pixelSensorLeft.getDistance(DistanceUnit.INCH);
//        }
        else {
            rightPixelSensorValue = 10;
            leftPixelSensorValue = 10;
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
