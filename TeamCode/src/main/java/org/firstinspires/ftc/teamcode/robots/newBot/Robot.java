package org.firstinspires.ftc.teamcode.robots.newBot;

public class Robot implements TelemetryProvider {

    // SUBSYSTEM INIT
    public final DriveTrainBase driveTrain;

    public Robot(HardwareMap hardwareMap)
    {
        driveTrain = new MeccanumDrive(hardwareMap);
    }

    

}
