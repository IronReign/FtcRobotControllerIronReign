package org.firstinspires.ftc.teamcode.robots.ri2d2;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem.Flywheel;
import org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem.drivetrain.DriveTrainBase;
import org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem.drivetrain.MecanumDrive;

@Config(value = "newBot_Robot")
public class Robot{

    // Sfalse = one motor, true = two motors
    public static boolean USE_TWO_MOTOR_FLYWHEEL = false;

    // SUBSYSTEMS DECLARATION
    public final DriveTrainBase driveTrain;
    public final Intake intake;
    //public final Flywheel flywheel;

    //SUBSYTEM ARRAY DECLARATION
    private final List<Subsystem> subsystems = new ArrayList<>();



    public Robot(HardwareMap hardwareMap)
    {
        // SBUSYSTEM INIT
        driveTrain = new MecanumDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        //flywheel = new Flywheel(hardwareMap, USE_TWO_MOTOR_FLYWHEEL);
        
        // ADD SUBSYSTEMS TO ARRAY
        subsystems.add(driveTrain);
        subsystems.add(intake);
        //subsystems.add(flywheel);
        
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
