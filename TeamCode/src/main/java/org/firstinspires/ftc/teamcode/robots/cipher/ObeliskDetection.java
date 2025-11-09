package org.firstinspires.ftc.teamcode.robots.cipher;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class ObeliskDetection extends LinearOpMode{
    private Limelight3A limelight;
    private Servo intake;

    private String[] pattern = {};
    private int step = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        intake = hardwareMap.get(Servo.class, "intakeServo");

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        int tagID = detectAprilTag();
        if (tagID = 21) {
            pattern = new String []{"PURPLE", "GREEN", "PURPLE"};
        } else if (tagID = 22) {
            pattern = new String[{""}]

        }


    }
}
