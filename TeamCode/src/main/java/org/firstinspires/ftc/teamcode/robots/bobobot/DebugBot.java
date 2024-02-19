package org.firstinspires.ftc.teamcode.robots.bobobot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.bobobot.Subsystems.RunnerBot;
import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Subsystem;

import java.util.LinkedHashMap;
import java.util.Map;

public class DebugBot implements Subsystem {
    public MotorDebug motorDebug;
    public Subsystem[] subsystems;
    public DriveTrain driveTrain;
    public HardwareMap hardwareMap;

    Telemetry telemetry;

    public DebugBot(MultipleTelemetry telemetry, HardwareMap hardwareMap){
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        driveTrain = new DriveTrain(hardwareMap, this);
        subsystems = new Subsystem[]{driveTrain};
        motorDebug = new MotorDebug(telemetry, hardwareMap);
    }

    public void Turn90(){
        driveTrain.turnUntilDegreesIMU(90, 2);
    }

    public void Turn0(){
        driveTrain.turnUntilDegreesIMU(0,2);
    }

    public void Turn180(){
        driveTrain.turnUntilDegreesIMU(180,2);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        return telemetryMap;
    }

    @Override
    public String getTelemetryName(){return "DEBUGBOT";}

    @Override
    public void update(Canvas fieldOverlay){
        driveTrain.updatePoseEstimate();

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

}
