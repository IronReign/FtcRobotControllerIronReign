package org.firstinspires.ftc.teamcode.robots.newBot.subystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MeccanumDrive {

    // MOTOR DECLARATION
    private final DcMotorEx frontRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx rearLeft;
    private final DcMotorEx rearRight;



    public MeccanumDrive(HardwareMap hardwareMap)
    {
        // MOTOR INIT
        frontRight = hardwareMap.get( DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get( DcMotorEx.class, "frontLeft");
        rearRight = hardwareMap.get( DcMotorEx.class, "rearRight");
        rearLeft = hardwareMap.get( DcMotorEx.class, "rearLeft");

        // REVERSE LEFT MOTORS
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // SET ZERO POWER BEHAVIOR
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void drive(double forward, double strafe, double turn)
    {
        
        double frontRightPower = forward - strafe - turn;
        double frontLeftPower  = forward + strafe + turn;
        double rearLeftPower   = forward - strafe + turn;
        double rearRightPower  = forward + strafe - turn;

        double max = Math.max( 1.0,
                Math.max( Math.max(Math.abs(frontLeftPower), Math.abs(rearLeftPower)),
                        Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));

        frontRightPower /= max;
        frontLeftPower /= max;
        rearRightPower /= max;
        rearLeftPower /= max;

        setMotorPowers( frontRightPower, frontLeftPower, rearRightPower, rearLeftPower);

    }

    private void setMotorPowers( double frontRightPower, double frontLeftPower, double rearRightPower, double rearLeftPower) {
        
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(rearLeftPower);
        rearRight.setPower(rearRightPower);
        rearLeft.setPower(rearLeftPower);
            
    }


    public void stop(){
        drive(0, 0, 0);
    }


}
