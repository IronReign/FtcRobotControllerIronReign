package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;

import static org.firstinspires.ftc.teamcode.robots.csbot.CenterStage_6832.alliance;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProviders;

import java.util.LinkedHashMap;
import java.util.Map;

public class RunnerBot implements Subsystem{
    public Subsystem[] subsystems;
    public DriveTrain driveTrain;
    public Intake intake;
    public Drone drone;
    public HardwareMap hardwareMap;
    public VisionProvider visionProvider = null;
    public static boolean visionOn = true;
    public boolean visionProviderFinalized = false;
    public static int visionProviderIndex = 2;

    Telemetry telemetry;
    public RunnerBot(MultipleTelemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        createVisionProvider();
        intake = new Intake(hardwareMap, this);
        driveTrain = new DriveTrain(hardwareMap, this);
        drone = new Drone(hardwareMap, this);
        subsystems = new Subsystem[]{driveTrain, intake, drone};
        visionProviderIndex = 2;
    }

    private static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Vision On/Vision Provider Finalized", visionOn+" "+visionProviderFinalized);
        telemetryMap.put("visionProvider name", visionProvider.getTelemetryName());
        return telemetryMap;
    }

    @Override
    public String getTelemetryName(){return "RUNNERBOT";}

    @Override
    public void update(Canvas fieldOverlay){
        driveTrain.updatePoseEstimate();
        drawRobot(fieldOverlay, driveTrain.pose);

        for (int i = 0; i < subsystems.length; i++){
            Subsystem subsystem = subsystems[i];
            subsystem.update(fieldOverlay);
        }
    }

    @Override
    public void stop(){
        for(Subsystem component : subsystems) {
            component.stop();
        }
    }

    public void createVisionProvider() {
        try {
            visionProvider = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance().setRedAlliance(alliance== Constants.Alliance.RED);
        } catch (IllegalAccessException | InstantiationException e) {
            throw new RuntimeException("Error while instantiating vision provider");
        }
    }

    public void enableVision() {
        if (visionOn) {
            if (!visionProviderFinalized) {
                createVisionProvider();
                visionProvider.initializeVision(hardwareMap, this);
                visionProviderFinalized = true;

            }
            visionProvider.update();
        }
    }

    public void start(){
        intake.closeClaw();
    }

}
