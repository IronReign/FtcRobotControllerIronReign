package org.firstinspires.ftc.teamcode.robots.csbot.vision;

import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.AprilTagProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.CSBotPropDetectorProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.OpenCVProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.PixelStackDetectorProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.dummy.LeftDummyProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.dummy.MiddleDummyProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.dummy.RightDummyProvider;

public class VisionProviders {
    public static final Class<? extends VisionProvider>[] VISION_PROVIDERS =
            new Class[]{AprilTagProvider.class,  PixelStackDetectorProvider.class, CSBotPropDetectorProvider.class, LeftDummyProvider.class, MiddleDummyProvider.class, RightDummyProvider.class,};
}
