package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

public class IntakeClaw {
    private DcMotorEx clawArm = null;
    private Servo clawSpan = null;
    public static double OPENCLAW = 0.15;
    public static double CLOSECLAW = 0.35;
    public static double armliftSpeed = 0.3;
    private static int armPosition;
    private static int armInitPosition = -129;
    private double powerArm = 0;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public IntakeClaw(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void telemetryOutput()
    {
        telemetry.addData("Claw Position \t", Utils.servoDenormalize(clawSpan.getPosition()));
        telemetry.addData("Claw Arm Position \t", clawArm.getCurrentPosition());
    }
    public void intakeClawInit()
    {
        clawArm = this.hardwareMap.get(DcMotorEx.class, "clawArm");
        clawSpan = this.hardwareMap.get(Servo.class, "clawSpan");
        //clawArm.setDirection(DcMotor.Direction.REVERSE);
        clawArm.setTargetPosition(armInitPosition);
        clawArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
    public void openClaw (boolean press)
    {
        if(press == true)
            clawSpan.setPosition(OPENCLAW);
    }
    public void closeClaw (boolean press)
    {
        if(press == true)
            clawSpan.setPosition(CLOSECLAW);

    }

    public void clawArmLift (double press)
    {
        press = -press;
        powerArm = 0.23*press;
        clawArm.setPower(armliftSpeed*powerArm);
    }

    public double getClawPosition () { return clawSpan.getPosition();}
    //adb connect 192.168.43.1
}

