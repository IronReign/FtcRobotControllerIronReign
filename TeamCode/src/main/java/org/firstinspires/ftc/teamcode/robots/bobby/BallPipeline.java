package org.firstinspires.ftc.teamcode.robots.bobby;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.opencv.core.*;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Imgproc;

import java.util.*;

public class BallPipeline extends OpenCvPipeline
{
    // outputs for auton
    public volatile double ballX = -1;
    public volatile double ballArea = 0;

    // for tuning
    static final boolean HSV_TUNER_ON = true;

    // noise reduction
    static final double MIN_CONTOUR_AREA = 800; // for small blobs
    static double MAX_ASPECT_RATIO = 2.0;       // skinny shapes
    static final double SMOOTHING_ALPHA = 0.7;  //
    private final Mat hsv = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5,5));

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // smoothed vals
    private double smoothedX = -1;
    private double smoothedArea = 0;

    @Override
    public Mat processFrame(Mat input)
    {
        // convert to HSV
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);

        // info about webcam
        int cameraWidth = input.cols();
        int cameraHeight = input.rows();
        int cameraCenterX = cameraWidth / 2;

        // --- DECLARE bestRect HERE for HSV tuner use ---
        Rect bestRect = null;
        double bestArea = 0;

        // Colors in HSV (HUE , SATURATION, VALUE)
        Scalar greenLower = new Scalar(35, 80, 80);
        Scalar greenUpper = new Scalar(85, 255, 255);
        Scalar purpleLower = new Scalar(125, 70, 70);
        Scalar purpleUpper = new Scalar(160, 255, 255);

        // Declare masks
        Mat greenMask = new Mat();
        Mat purpleMask = new Mat();
        Mat combinedMask = new Mat();

        Core.inRange(hsv, greenLower, greenUpper, greenMask);
        Core.inRange(hsv, purpleLower, purpleUpper, purpleMask);
        Core.bitwise_or(greenMask, purpleMask, combinedMask);

        // Morphology for noise cleaning
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(combinedMask, combinedMask, Imgproc.MORPH_CLOSE, kernel);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                combinedMask,
                contours,
                new Mat(),
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        // --- CONTOUR LOOP to find bestRect ---
        for (MatOfPoint c : contours)
        {
            double area = Imgproc.contourArea(c);
            if (area < MIN_CONTOUR_AREA) continue;

            Rect r = Imgproc.boundingRect(c);

            double perimeter = Imgproc.arcLength(new MatOfPoint2f(c.toArray()), true);
            double circularity = 4 * Math.PI * area / (perimeter * perimeter);

            double aspect = (double) r.width / r.height;
            if (aspect < 1.0) aspect = 1.0 / aspect;

            boolean semicircleAllowed = area > 2000;
            if (aspect > MAX_ASPECT_RATIO && !semicircleAllowed && circularity < 0.5) continue;

            if (area > bestArea)
            {
                bestArea = area;
                bestRect = r;
            }
        }

        // --- HSV TUNER ---
        // --- HSV TUNER ---
        if (HSV_TUNER_ON)
        {
            int size = 10;

            // 1️⃣ Camera center HSV
            int cx = cameraWidth / 2;
            int cy = cameraHeight / 2;

            Rect centerROI = new Rect(
                    Math.max(cx - size/2, 0),
                    Math.max(cy - size/2, 0),
                    Math.min(size, cameraWidth - (cx - size/2)),
                    Math.min(size, cameraHeight - (cy - size/2))
            );
            Mat centerRegion = hsv.submat(centerROI);
            Scalar centerAvg = Core.mean(centerRegion);
            centerRegion.release();

            // 2️⃣ Detected ball HSV (fallback to center if no ball)
            int ballSampleX = cx;
            int ballSampleY = cy;
            double ballAreaForTelemetry = 0;

            if (bestRect != null)
            {
                ballSampleX = bestRect.x + bestRect.width / 2;
                ballSampleY = bestRect.y + bestRect.height / 2;
                ballAreaForTelemetry = bestArea; // store detected ball area
            }

            Rect ballROI = new Rect(
                    Math.max(ballSampleX - size/2, 0),
                    Math.max(ballSampleY - size/2, 0),
                    Math.min(size, cameraWidth - (ballSampleX - size/2)),
                    Math.min(size, cameraHeight - (ballSampleY - size/2))
            );
            Mat ballRegion = hsv.submat(ballROI);
            Scalar ballAvg = Core.mean(ballRegion);
            ballRegion.release();

            // --- Send telemetry ---
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("HSV Center / H", centerAvg.val[0]);
            packet.put("HSV Center / S", centerAvg.val[1]);
            packet.put("HSV Center / V", centerAvg.val[2]);
            packet.put("HSV Ball / H", ballAvg.val[0]);
            packet.put("HSV Ball / S", ballAvg.val[1]);
            packet.put("HSV Ball / V", ballAvg.val[2]);
            packet.put("Ball Area", ballAreaForTelemetry);  // <-- added
            dashboard.sendTelemetryPacket(packet);

            // --- Draw markers ---
            Imgproc.drawMarker(input, new Point(cx, cy), new Scalar(0, 255, 0), Imgproc.MARKER_CROSS, 30, 2); // center
            Imgproc.drawMarker(input, new Point(ballSampleX, ballSampleY), new Scalar(0, 0, 255), Imgproc.MARKER_CROSS, 30, 2); // ball
        }

        // --- Draw camera center line ---
        Imgproc.line(
                input,
                new Point(cameraCenterX, 0),
                new Point(cameraCenterX, cameraHeight),
                new Scalar(255, 0, 0),
                2
        );

        // --- Ball detection result ---
        if (bestRect != null)
        {
            double rawX = bestRect.x + bestRect.width / 2.0;

            if (smoothedX < 0)
            {
                smoothedX = rawX;
                smoothedArea = bestArea;
            }
            else
            {
                smoothedX = SMOOTHING_ALPHA * smoothedX + (1.0 - SMOOTHING_ALPHA) * rawX;
                smoothedArea = SMOOTHING_ALPHA * smoothedArea + (1.0 - SMOOTHING_ALPHA) * bestArea;
            }

            ballX = smoothedX;
            ballArea = smoothedArea;

            Imgproc.rectangle(input, bestRect, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(ballX, bestRect.y + bestRect.height / 2.0), 5, new Scalar(0, 0, 255), -1);
        }


        else
        {
            ballX = -1;
            ballArea = 0;
            smoothedX = -1;
            smoothedArea = 0;
        }

        return input;
    }
}
