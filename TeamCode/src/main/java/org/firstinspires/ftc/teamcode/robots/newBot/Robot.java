package org.firstinspires.ftc.teamcode.robots.newBot;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.robots.newBot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.newBot.subsystem.drivetrain.DriveTrainBase;
import org.firstinspires.ftc.teamcode.robots.newBot.subsystem.drivetrain.MecanumDrive;

public class Robot{

    // SUBSYSTEMS DECLARATION
    public final DriveTrainBase driveTrain;

    //SUBSYTEM ARRAY DECLARATION
    private final List<Subsystem> subsystems = new ArrayList<>();



    public Robot(HardwareMap hardwareMap)
    {
        // SBUSYSTEM INIT
        driveTrain = new MeccanumDrive(hardwareMap);
        
        // ADD SUBSYSTEMS TO ARRAY
        subsystems.add(driveTrain);
        
    }


    public void update(Canvas fieldOverlay){
        
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

    public void resetStates(){
        for(Subsystem subsystem : subsystems)
            subsystem.resetStates();
    }
    

}
