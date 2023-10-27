package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

public class IntakeClaw {
    private DcMotorEx clawArm = null;
    private Servo clawSpan = null;
    public static double OPENCLAW = 0.2;
    public static double CLOSECLAW = 0.05;
    public static double armliftSpeed = 0.1;
    private static int armPosition;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public IntakeClaw(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void telemetryOutput()
    {
        telemetry.addData("Claw Span  \t", Utils.servoDenormalize(clawSpan.getPosition()));
        telemetry.addData("Claw Arm Position \t", Utils.servoDenormalize(clawArm.getCurrentPosition()));
    }
    public void intakeClawInit()
    {
        clawArm = this.hardwareMap.get(DcMotorEx.class, "clawArm");
        clawSpan = this.hardwareMap.get(Servo.class, "clawSpan");
    }
    public void openClaw () {clawSpan.setPosition(OPENCLAW);}
    public void closeClaw () {clawSpan.setPosition(CLOSECLAW);}

    public void clawArmLift (float position)
    {
        armPosition = armPosition + (int)(armliftSpeed*position);
        if(armPosition < 500)
            armPosition = 500;
        if(armPosition < 750)
            armPosition = 750;
        clawArm.setPositionPIDFCoefficients(Utils.servoNormalize(armPosition));
    }
    public double getClawPosition () { return clawSpan.getPosition();}
}

