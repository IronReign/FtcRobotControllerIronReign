package org.firstinspires.ftc.teamcode.robots.csbot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config(value = "pixelSensorTest")
@TeleOp(name = "Test Pixel Sensor", group = "Test")
public class TestPixelSensor extends LinearOpMode {
    public double rightPixelSensorValue, leftPixelSensorValue;
    DistanceSensor pixelSensorRight, pixelSensorLeft;
    public static boolean pixelSensorEnabled = true;

    public void runOpMode() {
        telemetry.update();
        pixelSensorRight = hardwareMap.get(DistanceSensor.class, "rightPixelSensor");
        pixelSensorLeft = hardwareMap.get(DistanceSensor.class, "leftPixelSensor");
        waitForStart();
        while(opModeIsActive())
        {
            telemetry.update();
            if(pixelSensorEnabled) {
                rightPixelSensorValue = pixelSensorRight.getDistance(DistanceUnit.INCH);
                leftPixelSensorValue = pixelSensorLeft.getDistance(DistanceUnit.INCH);
            }
            telemetry.addData("leftPixelSensor\t", leftPixelSensorValue);
            telemetry.addData("rightPixelSensor\t", rightPixelSensorValue);
        }
    }
}
