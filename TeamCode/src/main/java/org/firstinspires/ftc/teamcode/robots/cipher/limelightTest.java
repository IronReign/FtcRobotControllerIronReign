package org.firstinspires.ftc.teamcode.robots.cipher;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous

public class limelightTest extends OpMode {
    private Limelight3A limelight3A;

    @Override
    public void init(){
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(3);      //3 blue 4 red

    }

    @Override
    public void start(){
        limelight3A.start();
    }

    @Override
    public void loop(){
        LLResult llResult = limelight3A.getLatestResult();
        if(llResult!=null && llResult.isValid()){
            //Pose3D botpose = llResult.getBotpose();
            telemetry.addData("target x offset", llResult.getTx());
            telemetry.addData("target y offset", llResult.getTy());
            telemetry.addData("target area offset", llResult.getTa());
            //telemetry.addData("Botpose", botpose.toString());

        }
        telemetry.update();
    }
}
