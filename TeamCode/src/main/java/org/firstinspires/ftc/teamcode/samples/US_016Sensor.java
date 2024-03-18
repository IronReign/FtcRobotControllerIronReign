package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class US_016Sensor {

    public Telemetry telemetry;
    public AnalogInput analogInput;
    public DigitalChannel digitalChannel;
    public static DigitalChannel.Mode MODE = DigitalChannel.Mode.OUTPUT;
    public static double INCHES_PER_VOLT = 24.0;
    public US_016Sensor(HardwareMap hardwareMap, int analogPort, int digitalPort){
        analogInput = hardwareMap.get(AnalogInput.class, "us016leftAnalog");
        digitalChannel = hardwareMap.get(DigitalChannel.class, "us016left");
        digitalChannel.setMode(MODE);
        digitalChannel.setState(true);

    }

}
