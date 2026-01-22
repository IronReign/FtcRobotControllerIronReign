package org.firstinspires.ftc.teamcode.robots.bobby;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BOBBYAUTONR")
@Config(value = "BOBBYAUTON")

public class Auton extends OpMode
{

    Robot robot;
    FtcDashboard dashboard;

    public enum State {
        BACK_UP, SHOOT_BALLS, BACK_UP_MORE, ROTATE_1, INTAKE_1  , DONE
    }

    State autonState = State.BACK_UP;

    // Constants
    static final double CAMERA_CENTER_X = 320;
    static final double FLYWHEEL_POWER = 1.0;



    public static  double kP = 0.05;
    public static  double tol = 1.5;
    public static  double turnTarget = 90;




    // Backup/Strafe tracking
    private double backStartX;
    private double backStartY;
    private boolean driveInitialized = false;

    private double strafeStartX;
    private double strafeStartY;
    private boolean strafeInitialized = false;



    // Shooting sequence control
    private boolean shootInitialized = false;
    ElapsedTime timer = new ElapsedTime();
    int ballsCollected = 0;
    int ballsShot = 0;



    // Shooting sequence variables
    ElapsedTime timer1 = new ElapsedTime();
    int shootStep = 0;
    boolean sequenceRunning = false;
    boolean flywheelOn = false;

    // Starting position
    public double startX = 51.44;
    public double startY = 51.44;

    ElapsedTime cycleTimer = new ElapsedTime();
    double cycleTimeMs = 0;

    double error;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, null);
        robot.init();
        robot.x = startX;
        robot.y = startY;

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        cycleTimeMs = cycleTimer.milliseconds();
        cycleTimer.reset();
        robot.updatePose();
        executeAuton();
        sendDashboardTelemetry();
        drawOverlay();
    }

    private void executeAuton() {
        switch (autonState) {

            case BACK_UP:

                if(turnToHeading( Math.toRadians(turnTarget), kP, Math.toRadians(tol)) ) autonState = State.DONE;
                break;


            case DONE:
                stop();
                break;
        }
    }


    public boolean turnToHeading(double target, double kP, double tolerance)
    {

        error = target - robot.getHeading();

        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;


        if (Math.abs(error) <= tolerance) {
            robot.setTurnPower(0);
            return true;
        }

        double turnPower = error * kP;
        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));

        robot.setTurnPower(turnPower);
        return false;
    }


    private boolean driveBackwards(double distance, double power)
    {

        if (!driveInitialized) {
            backStartX = robot.x;
            backStartY = robot.y;
            driveInitialized = true;
        }

        double traveled = Math.hypot(robot.x - backStartX, robot.y - backStartY);

        if (traveled < distance) {
            robot.mecanumDrive(power, 0, 0); // BACKWARD
            return false;
        } else {
            robot.mecanumDrive(0, 0, 0);
            driveInitialized = false;
            return true;
        }
    }

    private boolean driveForward(double distance, double power)
    {
        if(!driveInitialized)
        {
            backStartX = robot.x;
            backStartY = robot.y;
            driveInitialized = true;
        }

        double traveled = Math.hypot(robot.x - backStartX, robot.y - backStartY);

        if (traveled < distance) {
            robot.mecanumDrive(-power, 0, 0); // FORWARD
            return false;
        } else {
            robot.mecanumDrive(0, 0, 0);
            driveInitialized = false;
            return true;
        }
    }

    private boolean strafeRight(double distance, double power) {
        if (!strafeInitialized) {
            strafeStartX = robot.x;
            strafeStartY = robot.y;
            strafeInitialized = true;
        }

        double traveled = Math.hypot(robot.x - strafeStartX, robot.y - strafeStartY);

        if (traveled < distance) {
            robot.mecanumDrive(0, -power, 0); // strafe right
            return false;
        } else {
            robot.mecanumDrive(0, 0, 0);
            strafeInitialized = false;
            return true;
        }
    }

    private boolean strafeLeft(double distance, double power) {
        if (!strafeInitialized) {
            strafeStartX = robot.x;
            strafeStartY = robot.y;
            strafeInitialized = true;
        }

        double traveled = Math.hypot(robot.x - strafeStartX, robot.y - strafeStartY);

        if (traveled < distance) {
            robot.mecanumDrive(0, power, 0); // strafe left
            return false;
        } else {
            robot.mecanumDrive(0, 0, 0);
            strafeInitialized = false;
            return true;
        }
    }
    public boolean shootingSequenceAuto()
    {
        sequence(true);
        return !sequenceRunning;
    }


    public void sequence(boolean start) {
        if (start && !sequenceRunning) {
            sequenceRunning = true;
            timer1.reset();
            shootStep = 0;
            flywheelOn = true;
            robot.setFlywheelPower(FLYWHEEL_POWER);
        }

        if (!sequenceRunning) return;

        double time = timer1.seconds();
        double PRELOAD_TIME = 0.5;
        double SERVO_UP_TIME = 0.2;
        double SERVO_DOWN_TIME = 0.4;
        double FEED_TIME = 0.4;
        double SHOT_TIME = SERVO_UP_TIME + SERVO_DOWN_TIME + FEED_TIME;

        if (time < PRELOAD_TIME) {
            robot.setIntakePower(0.3);
            robot.setPusherDown();
            return;
        }

        double shotTime = time - PRELOAD_TIME;
        double singleShotTime = shotTime - (shootStep * SHOT_TIME);

        if (shootStep < 3) {
            boolean keepIntakeOn = shootStep == 1;
            if (singleShotTime < SERVO_UP_TIME) {
                robot.setPusherUp();
                robot.setIntakePower(keepIntakeOn ? 0.8 : 0.0);
            } else if (singleShotTime < SERVO_UP_TIME + SERVO_DOWN_TIME) {
                robot.setPusherDown();
                robot.setIntakePower(keepIntakeOn ? 0.1 : 0.0);
            } else if (singleShotTime < SHOT_TIME) {
                robot.setIntakePower(1.0);
                robot.setPusherDown();
            } else {
                shootStep++;
            }
            return;
        }

        // Shooting finished
        robot.setIntakePower(0.0);
        robot.setPusherDown();
        robot.setFlywheelPower(0.0);
        sequenceRunning = false;
        flywheelOn = false;
        shootInitialized = false;
    }
    public void stop() {
        robot.mecanumDrive(0, 0, 0);
        robot.setIntakePower(0);
        robot.setFlywheelPower(0);
    }



    // ---------- Dashboard ----------
    private void sendDashboardTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Auton State", autonState.toString());
        packet.put("Robot X", robot.x);
        packet.put("Robot Y", robot.y);
        packet.put("Heading", Math.toDegrees(robot.getHeading()));
        packet.put("Balls Collected", ballsCollected);
        packet.put("Balls Shot", ballsShot);
        packet.put("Ball X", robot.pipeline.ballX);
        packet.put("Ball Area", robot.pipeline.ballArea);
        packet.put("error", error);
        packet.put("Cycle Time (ms)", cycleTimeMs);
        packet.put("Loop Frequency (Hz)", 1000.0 / cycleTimeMs);

        dashboard.sendTelemetryPacket(packet);
    }

    private void drawOverlay() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        canvas.setStroke("#00FF00");
        canvas.strokeCircle(0, 0, 5);

        if (robot.pipeline.ballX != -1) {
            double ballRelativeX = (robot.pipeline.ballX - CAMERA_CENTER_X) / 10.0;
            double radius = Math.max(2, robot.pipeline.ballArea / 1000.0);
            canvas.setStroke("#FF0000");
            canvas.strokeCircle(ballRelativeX, 0, radius);
        }

        dashboard.sendTelemetryPacket(packet);
    }
}

