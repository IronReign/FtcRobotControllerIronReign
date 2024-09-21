package org.firstinspires.ftc.teamcode.robots.deepthought.vision;

import org.firstinspires.ftc.teamcode.robots.deepthought.vision.provider.AprilTagProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.provider.CSBotPropDetectorProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.provider.OpenCVProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.provider.PixelStackDetectorProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.provider.dummy.LeftDummyProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.provider.dummy.MiddleDummyProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.provider.dummy.RightDummyProvider;

public class VisionProviders {
    public static final Class<? extends VisionProvider>[] VISION_PROVIDERS =
            new Class[]{AprilTagProvider.class,  PixelStackDetectorProvider.class, CSBotPropDetectorProvider.class, LeftDummyProvider.class, MiddleDummyProvider.class, RightDummyProvider.class, OpenCVProvider.class};
}
