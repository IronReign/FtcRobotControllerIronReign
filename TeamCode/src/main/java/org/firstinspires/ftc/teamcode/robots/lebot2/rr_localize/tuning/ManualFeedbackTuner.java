package org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.MecanumDriveReign;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.TankDrivePinpoint;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.rrQuickStart.TankDrive;
import org.firstinspires.ftc.teamcode.rrQuickStart.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.robots.lebot2.rr_localize.tuning.TuningOpModes;

public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDriveReign.class)) {
            MecanumDriveReign drive = new MecanumDriveReign(hardwareMap, new Pose2d(0, 0, 0));
            
//            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
//                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
//                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
//                }
//            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
//                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
//                    throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
//                }
//            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrivePinpoint.class)) {
            TankDrivePinpoint drive = new TankDrivePinpoint(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                runBlockingWithRefresh(drive,
                    drive.actionBuilder(new Pose2d(0, 0, 0))
                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());
            }
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
