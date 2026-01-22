package org.firstinspires.ftc.teamcode.robots.bobby;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BOBBYAUTONRED+CAM")
public class Auton2 extends OpMode {

    Robot robot;
    FtcDashboard dashboard;

    public enum State {
        BACK_UP, SHOOT_BALLS, INTAKE_SEQUENCE, GO_TO_CENTER, TURN_45, DRIVE_FORWARD, SHOOT_BALLS_AGAIN, DONE
    }

    State autonState = State.BACK_UP;

    // Constants
    static final double BACKUP_DISTANCE = 19.75; // inches
    static final double DRIVE_POWER = 0.35;
    static final double TURN_KP = 0.005;
    static final double STRAFE_KP = 0.003;
    static final double AREA_MIN = 3000;
    static final double CAMERA_CENTER_X = 320;
    static final double TURN_ANGLE = Math.toRadians(-45);
    static final double FORWARD_DISTANCE = (4*12)+5; // inches
    static final double FLYWHEEL_POWER = 1.0;
    static final double POSITION_TOLERANCE = 0.5;

    // Backup tracking
    private double backStartX;
    private double backStartY;
    private boolean backInitialized = false;

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

    @Override
    public void init() {
        robot = new Robot(hardwareMap, gamepad1);
        robot.init();
        robot.x = startX;
        robot.y = startY;

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {
        robot.updatePose(); // update x, y, heading using encoders
        executeAuton();
        sendDashboardTelemetry();
        drawOverlay();
    }

    private void executeAuton() {
        switch (autonState) {
            case BACK_UP:
                driveBackwards(BACKUP_DISTANCE);
                break;

            case SHOOT_BALLS:
            case SHOOT_BALLS_AGAIN:
                if (!shootInitialized) {
                    shootInitialized = true;
                    sequenceRunning = false; // allow fresh start
                }
                shootingSequenceAuto();
                break;

            case INTAKE_SEQUENCE:
                intake3Balls();
                break;

            case GO_TO_CENTER:
                goToCenter();
                break;

            case TURN_45:
                turn45();
                break;

            case DRIVE_FORWARD:
                driveForwardRobotCentric(FORWARD_DISTANCE);
                break;

            case DONE:
                stop();
                break;
        }
    }



    // ---------- ROBOT-CENTRIC MOVEMENT ----------

    private void driveBackwards(double distance) // good
    {
        if (!backInitialized) {
            backStartX = robot.x;
            backStartY = robot.y;
            backInitialized = true;
        }

        double traveled = Math.hypot(robot.x - backStartX, robot.y - backStartY);

        if (traveled < distance) {
            robot.mecanumDrive(DRIVE_POWER, 0, 0); // straight backward robot-centric
        } else {
            robot.mecanumDrive(0, 0, 0);
            backInitialized = false;
            ballsShot = 0;
            autonState = State.SHOOT_BALLS;
        }
    }
    // ---------- Shooting sequence ----------
    public void shootingSequenceAuto() //good
    {
        sequence(true); // auto-start sequence
    }

    public void sequence(boolean start) //good
    {
        if (start && !sequenceRunning) {
            sequenceRunning = true;
            timer1.reset();
            shootStep = 0;
            flywheelOn = true;
            robot.setFlywheelPower(FLYWHEEL_POWER);
        }

        if (!sequenceRunning) return;

        double time = timer1.seconds();
        double PRELOAD_TIME = 0.1;
        double SERVO_UP_TIME = 0.3;
        double SERVO_DOWN_TIME = 0.6;
        double FEED_TIME = 0.4;
        double SHOT_TIME = SERVO_UP_TIME + SERVO_DOWN_TIME + FEED_TIME;

        if (time < PRELOAD_TIME) {
            robot.setIntakePower(1.0);
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
                robot.setIntakePower(keepIntakeOn ? 0.9 : 0.0);
            } else if (singleShotTime < SHOT_TIME) {
                robot.setIntakePower(1.0);
                robot.setPusherDown();
            } else {
                shootStep++;
            }
            return;
        }

        // Shooting finished → next state
        robot.setIntakePower(0.0);
        robot.setPusherDown();
        robot.setFlywheelPower(0.0);
        sequenceRunning = false;
        flywheelOn = false;
        shootInitialized = false;

        if (autonState == State.SHOOT_BALLS) {
            autonState = State.INTAKE_SEQUENCE;
        } else if (autonState == State.SHOOT_BALLS_AGAIN) {
            autonState = State.DONE;
        }
    }



    private void driveForwardRobotCentric(double distance)
    {
        // Target position along current heading
        double targetX = robot.x + distance * Math.cos(robot.getHeading());
        double targetY = robot.y + distance * Math.sin(robot.getHeading());

        double errorX = targetX - robot.x;
        double errorY = targetY - robot.y;
        double error = Math.hypot(errorX, errorY);

        if (error > POSITION_TOLERANCE) {
            // Rotate field error into robot frame
            double cos = Math.cos(robot.getHeading());
            double sin = Math.sin(robot.getHeading());

            double forward = cos * errorX + sin * errorY;
            double strafe  = -sin * errorX + cos * errorY;

            double forwardPower = Math.max(-DRIVE_POWER, Math.min(DRIVE_POWER, forward * 0.1));
            double strafePower  = Math.max(-DRIVE_POWER, Math.min(DRIVE_POWER, strafe * 0.1));

            robot.mecanumDrive(forwardPower, strafePower, 0);
        } else {
            robot.mecanumDrive(0, 0, 0);
            ballsShot = 0;
            robot.setFlywheelPower(FLYWHEEL_POWER);
            autonState = State.SHOOT_BALLS_AGAIN;
        }
    }

    public void stop() {
        robot.mecanumDrive(0, 0, 0);
        robot.setIntakePower(0);
        robot.setFlywheelPower(0);
    }

    public void goToCenter() {
        double dx = -robot.x;
        double dy = -robot.y;
        double error = Math.hypot(dx, dy);

        if (error > POSITION_TOLERANCE) {
            double forward = Math.signum(dx) * DRIVE_POWER;
            double strafe = Math.signum(dy) * DRIVE_POWER;
            robot.mecanumDrive(forward, strafe, 0);
        } else {
            robot.mecanumDrive(0, 0, 0);
            autonState = State.TURN_45;
        }
    }

    public void turn45() {
        double errorAngle = TURN_ANGLE - robot.getHeading();
        if (Math.abs(errorAngle) > 0.01) {
            double turnPower = Math.max(-0.3, Math.min(0.3, errorAngle * TURN_KP * 100));
            robot.mecanumDrive(0, 0, turnPower);
        } else {
            robot.mecanumDrive(0, 0, 0);
            autonState = State.DRIVE_FORWARD;
        }
    }

    public void intake3Balls() {
        if (ballsCollected < 3) {
            double ballX = robot.pipeline.ballX;
            double ballArea = robot.pipeline.ballArea;

            if (ballX == -1) {
                robot.mecanumDrive(0, 0, -0.2);
            } else {
                double errorX = ballX - CAMERA_CENTER_X;
                double turnPower = Math.max(-0.3, Math.min(0.3, errorX * TURN_KP));
                double forwardPower = ballArea < AREA_MIN ? DRIVE_POWER : 0;
                double strafePower = -errorX * STRAFE_KP;

                robot.mecanumDrive(forwardPower, strafePower, turnPower);

                if (ballArea >= AREA_MIN) {
                    robot.setIntakePower(1.0);
                    if (timer.seconds() > 1.0) {
                        robot.setIntakePower(0);
                        ballsCollected++;
                        timer.reset();
                    }
                } else {
                    timer.reset();
                }
            }
        } else {
            robot.mecanumDrive(0, 0, 0);
            ballsCollected = 0;
            autonState = State.GO_TO_CENTER;
        }
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
