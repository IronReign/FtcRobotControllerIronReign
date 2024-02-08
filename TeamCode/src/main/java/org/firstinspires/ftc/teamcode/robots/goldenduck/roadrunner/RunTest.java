package org.firstinspires.ftc.teamcode.robots.goldenduck.roadrunner;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.csbot.rr_stuff.TankDrive;
import org.firstinspires.ftc.teamcode.robots.csbot.rr_stuff.tuning.TuningOpModes;

public final class RunTest extends LinearOpMode {
    public static double DISTANCE = 16;


    @Override
    public void runOpMode() throws InterruptedException {
        if (org.firstinspires.ftc.teamcode.robots.csbot.rr_stuff.tuning.TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            Pose2d startPose = drive.pose;
            Action a = new SequentialAction(
                    drive.actionBuilder(startPose)
                            .lineToX(-20)
                            .build()
            );
            waitForStart();
            while (opModeIsActive()) {
               if(!a.run(new TelemetryPacket()))
                   throw new InterruptedException();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(DISTANCE)
                                .lineToX(0)
                                .build());
            }
        } else {
            throw new AssertionError();
        }
    }
}
