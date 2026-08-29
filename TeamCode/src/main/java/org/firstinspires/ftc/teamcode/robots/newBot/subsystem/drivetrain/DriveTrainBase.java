package org.firstinspires.ftc.teamcode.robots.newBot.subsystem.drivetrain;
import org.firstinspires.ftc.teamcode.robots.newBot.subsystem.Subsystem;

public interface DriveTrainBase extends Subsystem {

    void drive( double forward, double strafe, double turn );
}