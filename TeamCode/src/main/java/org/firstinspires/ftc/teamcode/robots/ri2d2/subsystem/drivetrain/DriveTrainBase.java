package org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem.drivetrain;
import org.firstinspires.ftc.teamcode.robots.ri2d2.subsystem.Subsystem;

public interface DriveTrainBase extends Subsystem {
    void drive( double forward, double strafe, double turn );
}