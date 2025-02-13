package org.firstinspires.ftc.teamcode.robots.deepthought.vision;

import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;
import static org.firstinspires.ftc.teamcode.util.utilMethods.isPast;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.robots.csbot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.deepthought.subsystem.Robot;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

@Config(value = "00_ITD_DEMOS")
public class Demos implements TelemetryProvider {
    Robot robot;

    public Demos(Robot robot) {
        this.robot = robot;
        demo = Demo.SCOOP;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> map = new LinkedHashMap<>();
        map.put("state", demo.name());
        map.put("scoopIndex", scoopIndex);
        return map;
    }

    @Override
    public String getTelemetryName() {
        return "DEMO";
    }

    enum Demo {
        SCOOP,

    }

    Demo demo;

    public boolean execute() {
        switch (demo) {
            case SCOOP:
                scoop();
                break;
        }


        return false;
    }

    public static int scoopIndex = 0;
    public long scoopTimer = 0;

    public boolean scoop() {
        switch (scoopIndex) {
            case 0:

                 if(robot.alignOnSample()) {
                     robot.driveTrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                     scoopIndex ++;
                 }
                break;
            case 1:
                robot.articulate(Robot.Articulation.SAMPLER_INTAKE);
                scoopIndex ++;
                break;
            case 2:
                if (robot.articulation == Robot.Articulation.MANUAL) {
                    robot.trident.sampler.servoPower = 0;
                    robot.articulate(Robot.Articulation.TRAVEL);
                    scoopIndex = 0;
                }
                break;
        }
        return false;

    }

}
