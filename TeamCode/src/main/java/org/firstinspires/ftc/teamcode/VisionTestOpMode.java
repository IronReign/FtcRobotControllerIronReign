package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.AVERAGE_LOOP_TIME_SMOOTHING_FACTOR;

import android.graphics.Color;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.util.FTCPanels;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;

@TeleOp()
@Config(value = "visionTest")
public class VisionTestOpMode extends OpMode {
    public VisionProvider visionProviderBack, visionProviderFront;
    FtcDashboard dashboard;
    public static int backVisionProviderIndex = 0;
    public static int frontVisionProviderIndex = 0;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private ExponentialSmoother averageUpdateTimeSmoother;
    public static boolean frontEnabled, backEnabled;
    public static boolean frontFinalized;
    public static boolean backFinalized;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        averageUpdateTimeSmoother = new ExponentialSmoother(AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);

        backFinalized = false;
        backEnabled = false;
        frontFinalized = false;
        frontEnabled = false;
        dashboard = FtcDashboard.getInstance();


    }

    @Override
    public void start() {
        elapsedTime.reset();
    }


    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        TelemetryPacket p = new TelemetryPacket();
        long updateStartTime = System.nanoTime();
        if (frontEnabled) {
            if (frontFinalized) {
                visionProviderFront.update(true);
            } else {
                try {
                    visionProviderFront = VisionProviders.VISION_PROVIDERS[frontVisionProviderIndex].newInstance();
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
                visionProviderFront.initializeVision(hardwareMap, new Robot(hardwareMap, true), true);
                frontFinalized = true;
            }
        }

        if(backEnabled) {
            if (backFinalized) {
                visionProviderBack.update(true);
            } else {
                try {
                    visionProviderBack = VisionProviders.VISION_PROVIDERS[backVisionProviderIndex].newInstance();
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
                visionProviderBack.initializeVision(hardwareMap, new Robot(hardwareMap, true), false);
                backFinalized = true;
            }
        }


        long updateTime = (System.nanoTime() - updateStartTime);
        double averageUpdateTime = averageUpdateTimeSmoother.update(updateTime);
        p.put("Average Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageUpdateTime * 1e-6), (int) (1 / (averageUpdateTime * 1e-9))));
        telemetry.addData("Average Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageUpdateTime * 1e-6), (int) (1 / (averageUpdateTime * 1e-9))));
        dashboard.sendTelemetryPacket(p);
    }

    @Override
    public void stop() {

    }
}