package org.firstinspires.ftc.teamcode.robots.deepthought.vision.provider;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.Position;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.vision.pipeline.OpenCVPipeline;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Arrays;
import java.util.Map;

/**
 * @author Mahesh Natamai
 */

@Config(value = "NEW OPENCV PROV")
public class OpenCVProvider extends VisionProvider {
    private Bitmap noCameraBitmap;
    private OpenCvCamera camera;
    private OpenCVPipeline pipeline;
    private volatile boolean cameraOpened;

    // Constants
    private static final String TELEMETRY_NAME = "OpenCV Vision Provider";
    public static int WEBCAM_WIDTH = 640;
    public static int WEBCAM_HEIGHT = 480;

    @Override
    public void initializeVision(HardwareMap hardwareMap, boolean isFront) {
        pipeline = new OpenCVPipeline();
        if(!isFront) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }
        else camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"));
        
        noCameraBitmap = Bitmap.createBitmap(320, 240, Bitmap.Config.RGB_565);
        Mat noCameraMat = new Mat(240, 320, CvType.CV_8UC3);
        Imgproc.putText(noCameraMat, "Webcam Could", new Point(40, 110), Imgproc.FONT_HERSHEY_SIMPLEX,
                1, new Scalar(255, 0, 0), 3);
        Imgproc.putText(noCameraMat, "Not Be Opened", new Point(40, 150), Imgproc.FONT_HERSHEY_SIMPLEX,
                1, new Scalar(255, 0, 0), 3);
        Utils.matToBitmap(noCameraMat, noCameraBitmap);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                cameraOpened = true;
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void shutdownVision() {
//        if(cameraOpened) {
            try {
                camera.stopStreaming();
                camera.closeCameraDevice();
            } catch(Exception e) {

            }
//        }
//        cameraOpened = false;
    }

    @Override
    public Position getPosition() {
        return cameraOpened ? pipeline.getLastPosition() : Position.HOLD;
    }

    @Override
    public void reset() {

    }

    @Override
    public boolean canSendDashboardImage() {
        return true;
    }

    @Override
    public Bitmap getDashboardImage() {
        return cameraOpened ? pipeline.getDashboardImage() : noCameraBitmap;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = super.getTelemetry(debug);

        if(debug && cameraOpened) {
            telemetryMap.put("Frame Count", camera.getFrameCount());
            telemetryMap.put("FPS", Misc.formatInvariant("%.2f", camera.getFps()));
            telemetryMap.put("Total frame time ms", camera.getTotalFrameTimeMs());
            telemetryMap.put("Pipeline time ms", camera.getPipelineTimeMs());
            telemetryMap.put("Overhead time ms", camera.getOverheadTimeMs());
            telemetryMap.put("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
            telemetryMap.put("Largest Coordinate", Arrays.toString(pipeline.getLargestCoordinate()));
        }

        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public void updateVision() {

    }
}
