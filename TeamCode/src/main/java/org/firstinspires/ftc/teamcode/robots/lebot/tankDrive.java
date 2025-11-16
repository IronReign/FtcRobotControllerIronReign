package org.firstinspires.ftc.teamcode.robots.lebot;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class tankDrive {

    private DcMotor leftFront, rightFront;

    public void init(HardwareMap hardwareMap){
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void drive(double throttle, double spin) {
        double leftPower = throttle + spin;
        double rightPower = throttle - spin;
        double largest = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if(largest > 1.0) {
            leftPower /= largest;
            rightPower /= largest;
        }

        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
    }

    public void power(double output){
        leftFront.setPower(-output);
        rightFront.setPower(output);
    }
}
