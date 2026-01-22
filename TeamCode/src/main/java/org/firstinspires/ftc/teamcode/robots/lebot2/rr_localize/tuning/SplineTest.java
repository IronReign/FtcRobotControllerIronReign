package org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.MecanumDriveReign;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDrivePinpoint;
import org.firstinspires.ftc.teamcode.rrQuickStart.TankDrive;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.tuning.TuningOpModes;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDriveReign.class)) {
            MecanumDriveReign drive = new MecanumDriveReign(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrivePinpoint.class)) {
            TankDrivePinpoint drive = new TankDrivePinpoint(hardwareMap, beginPose);

            waitForStart();

            runBlockingWithRefresh(drive,
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }

    /**
     * Custom runBlocking that manages Pinpoint localizer refresh cycles.
     * This ensures the localizer gets fresh I2C data each iteration.
     */
    private void runBlockingWithRefresh(TankDrivePinpoint drive, Action action) {
        PinpointLocalizer localizer = (PinpointLocalizer) drive.localizer;
        FtcDashboard dash = FtcDashboard.getInstance();

        while (!Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            localizer.refresh();
            boolean running = action.run(packet);
            localizer.markCycleComplete();
            dash.sendTelemetryPacket(packet);
            if (!running) break;
        }
    }
}
