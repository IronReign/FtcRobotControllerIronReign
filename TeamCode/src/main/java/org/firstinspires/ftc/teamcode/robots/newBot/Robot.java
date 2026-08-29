package org.firstinspires.ftc.teamcode.robots.newBot;

public class Robot extends TelemetryProvider {

    // SUBSYSTEM INIT
    public final MeccanumDrive driveTrain;

    public NewBot(HardwareMap hardwareMap)
    {
        MeccanumDrive = new MeccanumDrive(hardwareMap);
    }

    

}
