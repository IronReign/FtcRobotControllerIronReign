package org.firstinspires.ftc.teamcode.robots.bobby;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class MasterVisionPipeline extends OpenCvPipeline
{
    // Sub-pipelines
    private final BallPipeline ballPipeline;
    private final AprilTagDetectionPipeline aprilTagPipeline;

    // Shared outputs
    public volatile double ballX = -1;
    public volatile double ballArea = 0;
    private ArrayList<AprilTagDetection> aprilTagDetections = new ArrayList<>();

    public MasterVisionPipeline(
            double tagSize,
            double fx, double fy,
            double cx, double cy
    )
    {
        ballPipeline = new BallPipeline();
        aprilTagPipeline = new AprilTagDetectionPipeline(
                tagSize, fx, fy, cx, cy
        );
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Run ball detection
        Mat out = ballPipeline.processFrame(input);

        // Run AprilTag detection on same frame
        out = aprilTagPipeline.processFrame(out);

        // Copy outputs
        ballX = ballPipeline.ballX;
        ballArea = ballPipeline.ballArea;
        aprilTagDetections = aprilTagPipeline.getLatestDetections();

        return out;
    }

    // ---------- Accessors ----------
    public ArrayList<AprilTagDetection> getAprilTags()
    {
        return aprilTagDetections;
    }

    public double getBallX()
    {
        return ballX;
    }

    public double getBallArea()
    {
        return ballArea;
    }

    public void setAprilTagDecimation(float decimation)
    {
        aprilTagPipeline.setDecimation(decimation);
    }
}
