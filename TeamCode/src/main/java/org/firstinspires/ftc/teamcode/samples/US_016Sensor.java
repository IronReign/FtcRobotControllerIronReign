package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class US_016Sensor {

    public Telemetry telemetry;
    public AnalogInput analogInputLeft, analogInputRight;
    public DigitalChannel digitalChannelLeft, digitalChannelRight;
    public static DigitalChannel.Mode MODE = DigitalChannel.Mode.OUTPUT;
    public static double INCHES_PER_VOLT = 24.0;
    public US_016Sensor(HardwareMap hardwareMap){
        analogInputLeft = hardwareMap.get(AnalogInput.class, "us016leftAnalog");
        digitalChannelLeft = hardwareMap.get(DigitalChannel.class, "us016left");
        analogInputRight = hardwareMap.get(AnalogInput.class, "us016rightAnalog");
        digitalChannelRight = hardwareMap.get(DigitalChannel.class, "us016right");
        digitalChannelLeft.setMode(MODE); digitalChannelRight.setMode(MODE);
        digitalChannelLeft.setState(true); digitalChannelRight.setState(true);

    }

}
