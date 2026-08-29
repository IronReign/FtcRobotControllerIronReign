package org.firstinspires.ftc.teamcode.robots.newBot;

public class Robot implements TelemetryProvider {

    // SUBSYSTEMS DECLARATION
    private final DriveTrainBase driveTrain;

    //SUBSYTEM ARRAY DECLARATION
    private final List<Subsystem> subsystems = new ArrayList<>();



    public Robot(HardwareMap hardwareMap)
    {
        // SBUSYSTEM INIT
        driveTrain = new MeccanumDrive(hardwareMap);
        
        // ADD SUBSYSTEMS TO ARRAY
        subsystems.add(driveTrain);
        
    }


    public void update(){
        
        // I2C SENSOR READ 
        for(Subsystem subsystem : subsystems)
            subsystem.readSensors();

        // RUN CALCULATIONS
        for (Subsystem subsystem : subsystems) 
            subsystem.calc(fieldOverlay);
        
        //FLUSH PENDING COMMANDS
        for (Subsystem subsystem : subsystems) 
            subsystem.act();
        
    }


    public void stop()
    {
        for(Subsystem subsystem : subsystems)
            subsystem.stop();
    }

    public void resetStates({
        for(Subsystem subsystem : subsystems)
            subsystem.resetStates();
    }
    

}
