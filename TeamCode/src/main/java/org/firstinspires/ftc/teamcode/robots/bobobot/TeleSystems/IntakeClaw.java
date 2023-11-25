package org.firstinspires.ftc.teamcode.robots.bobobot.TeleSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.r2v2.util.Utils;

public class IntakeClaw {
    private DcMotorEx clawArm = null;
    private Servo clawSpan = null;
    private Servo clawWrist = null;
    public static double OPENCLAW = 0.15;
    public static double CLOSECLAW = 0.30;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public IntakeClaw(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    public void telemetryOutput()
    {
        telemetry.addData("Claw Position \t", Utils.servoDenormalize(clawSpan.getPosition()));
        telemetry.addData("Claw Values \t", clawSpan.getPosition());
        telemetry.addData("Claw Arm Position \t", clawArm.getCurrentPosition());
        telemetry.addData("Arm Speed \t", clawArm.getVelocity());
        telemetry.addData("Claw Wrist Position \t", Utils.servoDenormalize(clawWrist.getPosition()));
        telemetry.addData("Arm Target \t", clawArm.getTargetPosition());
    }
    public void intakeClawInit()
    {
        clawArm = this.hardwareMap.get(DcMotorEx.class, "clawArm");
        clawSpan = this.hardwareMap.get(Servo.class, "clawSpan");
        clawWrist = this.hardwareMap.get(Servo.class, "clawWrist");
        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setPower(1);
        clawArm.setVelocity(100);
        clawArm.setTargetPosition(20);
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
    public void clawArmLift (boolean press)
    {
        if(press == true) {
            clawArm.setTargetPosition(240);
        }
    }
    public void clawArmLower (boolean press)
    {
        if(press == true) {
            clawArm.setTargetPosition(20);
        }
    }
    public void armWristDown(boolean press) {
        if (press == true && clawArm.getCurrentPosition() < 200) {
            clawWrist.setPosition(0.43);
            clawArm.setTargetPosition(3);
        }
        else if (press == true && clawArm.getCurrentPosition() > 200) {
            clawWrist.setPosition(0.3);
        }
    }
    public void armWristUp(boolean press) {
        if (press == true) {
            clawWrist.setPosition(0);
        }
    }
    public void armPositionTest() {
        clawArm.setTargetPosition(90);
    }
}

