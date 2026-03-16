package org.firstinspires.ftc.teamcode.robots.bobby;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.util.StickyGamepad;

import java.util.LinkedHashMap;
import java.util.Map;

@TeleOp(name="bobby", group="game")
public class bobbyOpMode extends OpMode
{
    Robot robot;
    StickyGamepad g1=null;
    ElapsedTime timer1;
    boolean sequenceRunning;
    boolean sequence2Running;
    int shootStep = 0;
    //----------------------------------------------------------------------
    boolean flywheelOn = false;
    boolean intakeOn = false;
    @Override
    public void init(){
        // ----------INITIALIZE ROBOT
        robot=new Robot(hardwareMap, gamepad1,0);
        robot.init();
        g1=new StickyGamepad(gamepad1);

    // -------- VARIABLES FOR SHOOTING SEQUENCE
        timer1 = new ElapsedTime();
        sequenceRunning = false;



    }
    public void init_loop() {

    }

    @Override
    public void loop(){
        robot.update(new Canvas());
        g1.update();

        sequence(gamepad1);

        if(!sequenceRunning && !sequence2Running)
        {
            handlePusher(gamepad1);
            handleIntake(gamepad1);
            handleFlywheel(g1);
        }

        handleTelemetry(robot.getTelemetry(true), robot.getTelemetryName());
    }





    public void sequence(Gamepad gamepad)
    {
//g
        // -------- START SEQUENCE --------
        if (gamepad.y && !sequenceRunning) {
            sequenceRunning = true;
            timer1.reset();
            shootStep = 0;
            flywheelOn = true;
            robot.setFlywheelVelocity(125);
        }

        if (!sequenceRunning) return;

        double time = timer1.seconds();

        // -------- TIMING CONSTANTS --------
        double PRELOAD_TIME    = 0.5;  // intake before first shot // was0.1
        double SERVO_UP_TIME   = 0.2;  // servo moving up
        double SERVO_DOWN_TIME = 0.4;  // servo moving down
        double FEED_TIME       = 0.4;  // intake feeds next ball
        double SHOT_TIME       = SERVO_UP_TIME + SERVO_DOWN_TIME + FEED_TIME;

        // -------- PRELOAD BEFORE FIRST SHOT --------
        if (time < PRELOAD_TIME) {
            robot.setIntakePower(0.3);//was 1.0
            robot.setPusherDown();
            return;
        }

        // -------- TIME SINCE SHOOTING STARTED --------
        double shotTime = time - PRELOAD_TIME;
        double singleShotTime = shotTime - (shootStep * SHOT_TIME);

        // -------- SHOOTING LOGIC --------
        if (shootStep < 3) {
            // Determine if intake should stay on (for second shot)
            boolean keepIntakeOn = (shootStep == 1);

            // PUSHER UP — shoot
            if (singleShotTime < SERVO_UP_TIME) {
                robot.setPusherUp();
                robot.setIntakePower(keepIntakeOn ? 0.8 : 0.0);
            }
            // PUSHER DOWN — fully retract
            else if (singleShotTime < SERVO_UP_TIME + SERVO_DOWN_TIME) {
                robot.setPusherDown();
                robot.setIntakePower( keepIntakeOn ? 1.0 : 0.0);//worked w/ 0.9
            }
            // FEED NEXT BALL — intake runs
            else if (singleShotTime < SHOT_TIME) {
                robot.setIntakePower(1.0);
                robot.setPusherDown();
            }
            // NEXT SHOT
            else {
                shootStep++;
            }
            return;
        }

        // -------- SEQUENCE COMPLETE — RESET --------
        robot.setIntakePower(0.0);
        robot.setPusherDown();
        robot.setFlywheelVelocity(0.0);
        sequenceRunning = false;
        flywheelOn = false;
    }

    public void handlePusher(Gamepad gamepad){
        if (gamepad.b) robot.setPusherUp();
        else robot.setPusherDown();
    }
    public void handleIntake(Gamepad gamepad) {
        if(gamepad.right_trigger>0.0) robot.setIntakePower(1.0);

        else if (gamepad.left_bumper) robot.setIntakePower(-1.0);

        else robot.setIntakePower(0.0);
    }
    public void handleFlywheel(StickyGamepad g1) {
        if(g1.right_bumper)flywheelOn = !flywheelOn;

        if(flywheelOn) robot.setFlywheelVelocity(100);
        else robot.setFlywheelVelocity(0.0);
    }



    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName) {
        telemetry.addLine(telemetryName);
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            telemetry.addLine(line);
        }
        telemetry.addLine();
    }
    public Map<String, Object> getTelemetry(boolean debug) {
        LinkedHashMap<String,Object> telemetry=new LinkedHashMap<>();
        return telemetry;
    }

}