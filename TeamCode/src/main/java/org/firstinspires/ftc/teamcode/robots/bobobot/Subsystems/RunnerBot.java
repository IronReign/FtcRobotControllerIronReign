package org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.BobotPosition;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.bobobot.Utilities.PositionCache;

import java.util.LinkedHashMap;
import java.util.Map;

public class RunnerBot implements Subsystem{
    public Subsystem[] subsystems;
    public DriveTrain driveTrain;
    public Intake intake;
    public Drone drone;
    public HardwareMap hardwareMap;
    public BobotPosition currentPosition;
    public BobotPosition fetchedPosition;
    public boolean fetched;
    public static boolean updatePositionCache = false;
    public PositionCache positionCache;

    Telemetry telemetry;
    public RunnerBot(MultipleTelemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        //positionCache = new PositionCache(5);
        intake = new Intake(hardwareMap, this);
        driveTrain = new DriveTrain(hardwareMap, this);
        drone = new Drone(hardwareMap, this);
        subsystems = new Subsystem[]{driveTrain, intake, drone};
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
        return telemetryMap;
    }

    @Override
    public String getTelemetryName(){return "RUNNERBOT";}

    @Override
    public void update(Canvas fieldOverlay){
//        if (updatePositionCache){
//            currentPosition = new BobotPosition(driveTrain.pose);
//            positionCache.update(currentPosition, false);
//        }
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

    public void fetchCachedCSPosition() {
        fetchedPosition = positionCache.readPose();
        fetched = fetchedPosition != null;
    }
}
