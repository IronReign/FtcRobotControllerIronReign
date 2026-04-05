package org.firstinspires.ftc.teamcode.robots.bobby;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.util.PIDController;


@Autonomous(name = "BobbyAutonLEAVE")
@Config(value = "BobbyAutonLeave")


public class AutonLEAVE extends OpMode
{

    public static boolean BlueAlliance = true;


    // Robot and Dashboard Declaration
    Robot robot;
    FtcDashboard dashboard;

    // Auton Stages
    public enum State {
        BACK_UP, LAUNCH, BACK_UP_MORE, TURN_TO_INTAKE, INTAKE, FORTH_FROM_INTAKE, TURN_TO_LAUNCH, FORTH_TO_LAUNCH, STRAFE_TO_EXIT, BACK_UP_MORE2, TURN_TO_0, BACK_TO_ALIGN, DONE
    }
    State autonState = State.BACK_UP;


    // Constants
    static final double CAMERA_CENTER_X = 320;
    static final double FLYWHEEL_POWER = 1.0;


    // Backup/Strafe tracking
    private double backStartX;
    private double backStartY;
    private boolean driveInitialized = false;
    private double strafeStartX;
    private double strafeStartY;
    private boolean strafeInitialized = false;

    // Shooting sequence control + variables
    private boolean shootInitialized = false;
    ElapsedTime timer1 = new ElapsedTime();
    int shootStep = 0;
    boolean sequenceRunning = false;
    boolean flywheelOn = false;




    // cycle timer
    ElapsedTime cycleTimer = new ElapsedTime();
    double cycleTimeMs = 0;
    double error;
    double correction;


    //PID vars
    PIDController turnPID;
    PIDController xPID;
    PIDController yPID;


    PIDController driveDistPID;
    private boolean drivePIDinitialized;
    private double driveStartX, driveStartY;


    //kD for PIDs
    public static double drivekP = 1.5;
    public static double turnkP = 2.0 ;
    public static double turnD = 0.003;
    public static double turnTol = 0.025;


    public static double turnDeadZone = 1.5;
    private boolean turnInitialized = false;
    private double lastTurnError = 0;

    public static double intakeDist = 45;
    //    public static double drivePow = 0.7;
    private double[] intakePosY =  {2, 36, 61};
    private int shootNum = 0;






    // Starting position + constants per alliance
    public static double startX = 51.44;
    public static double startY = 51.44;
    public double launchX =  37.47 ;
    public double launchY = 37.47;
    public static double intakeAngle =  90 ;
    public static double firstBackUpDist = 12;
    public static double secondBackUpDist = 45.5 - firstBackUpDist;
    public static double launchHeading =  -45;
    public boolean zeroToLaunch = false;


    public static double backPow = 0.15;


    public static double maxTurnTime = 2800;


    ElapsedTime turnTimer = new ElapsedTime();


    ElapsedTime intakeTimer = new ElapsedTime();




    @Override
    public void init() {
        robot = new Robot( hardwareMap, Math.toRadians(launchHeading));
        robot.setPose(startX, startY);
        dashboard = FtcDashboard.getInstance();


        // Turn PID init
        turnPID = new PIDController(
                turnkP,   // P
                0.0,   // I
                turnD   // D
        );


        turnPID.setInputRange(0, 360);
        turnPID.setContinuous(true);
        turnPID.setOutputRange(-1.0, 1.0);
        turnPID.setIntegralCutIn(Math.toRadians(5));
        turnPID.setTolerance(Math.toRadians(turnTol));
        turnPID.enable();


        // drive pid init
        xPID = new PIDController(
                drivekP,  // P
                0.0,   // I
                0.0    // D
        );


        yPID = new PIDController(
                drivekP,  // P
                0.0,   // I
                0.0    // D
        );


        xPID.setOutputRange(-1.0, 1.0);
        yPID.setOutputRange(-1.0,1.0);
        xPID.setTolerance(1.5);
        yPID.setTolerance(1.5);
        xPID.enable();
        yPID.enable();




        driveDistPID = new PIDController(
                drivekP,
                0.0, 0.015


        );


        driveDistPID.setOutputRange(-1.0,1.0);
        driveDistPID.setTolerance(1.5);
        driveDistPID.enable();
    }


    @Override
    public void loop() {
        cycleTimeMs = cycleTimer.milliseconds();
        cycleTimer.reset();
        robot.update(new Canvas());
        executeAuton();
        sendDashboardTelemetry();
    }


    private void executeAuton() {
        switch (autonState) {


            case BACK_UP:
                if(driveBackwards(firstBackUpDist, 0.4)) autonState = State.LAUNCH;
                break;


            case LAUNCH:
                if(shootingSequenceAuto())
                {
                    autonState = State.STRAFE_TO_EXIT;
                }
                break;


            case STRAFE_TO_EXIT:
                if(BlueAlliance){ if(strafeRight(20, 1.0)) autonState = State.DONE;}
                else { if(strafeLeft(20,1.0)) autonState = State.DONE; }




            case DONE:
                stop();
                break;
        }
    }




    public boolean goToPID(double targetX, double targetY, double targetHeading) {


        // --- Position error ---
        double xError = targetX - robot.getX();
        double yError = targetY - robot.getY();


        xPID.setSetpoint(0);
        xPID.setInput(-xError);
        double xPower = xPID.performPID();


        yPID.setSetpoint(0);
        yPID.setInput(-yError);
        double yPower = yPID.performPID();


        // --- TURN PID (MATCH pidTurnToHeading CONFIG) ---
        double heading = robot.getHeading();


        turnPID.setInputRange(-Math.PI, Math.PI);
        turnPID.setOutputRange(-1, 1);
        turnPID.setContinuous(true);
        turnPID.setPID(new PIDCoefficients(turnkP, 0, turnD));
        turnPID.setTolerance(turnTol);


        turnPID.setSetpoint(targetHeading);
        turnPID.setInput(heading);


        double turnPower = -turnPID.performPID(); // SAME sign as pidTurnToHeading


        // --- Done check ---
        if (xPID.onTarget() && yPID.onTarget() && turnPID.onTarget()) {
            robot.mecanumDrive(0, 0, 0);
            return true;
        }


        // --- Field-centric → robot-centric ---
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);


        double robotForward = yPower * cos + xPower * sin;
        double robotStrafe  = xPower * cos - yPower * sin;


        // --- Normalize ---
        double max = Math.max(1.0,
                Math.max(Math.abs(robotForward),
                        Math.max(Math.abs(robotStrafe), Math.abs(turnPower))));


        robot.mecanumDrive(
                robotForward / max,
                robotStrafe / max,
                turnPower / max
        );


        return false;
    }


    public boolean pidTurnToHeading(double targetHeading) {


        double heading = robot.getHeading();


        turnPID.setInputRange(-Math.PI, Math.PI);
        turnPID.setOutputRange(-.8,.8);
        turnPID.setContinuous(true);
        turnPID.setSetpoint(targetHeading);
        turnPID.setInput(heading);
        PIDCoefficients HEADING_PID = new PIDCoefficients(turnkP, 0, turnD);
        turnPID.setPID(HEADING_PID);
        turnPID.setTolerance(turnTol);
        correction = -turnPID.performPID();
        robot.setTurnPower(correction);
        error = turnPID.getError();


        if(turnPID.onTarget())
        {
            robot.setTurnPower(0);
            return true;
        }




        return false;
    }


    private boolean driveBackwards(double distance, double power) {


        if (!driveInitialized) {
            backStartX = robot.getX();
            backStartY = robot.getY();
            driveInitialized = true;
        }


        double traveled = Math.hypot(robot.getX() - backStartX, robot.getY() - backStartY);


        if (traveled < distance) {
            robot.mecanumDrive(power, 0, 0); // BACKWARD
            return false;
        } else {
            robot.mecanumDrive(0, 0, 0);
            driveInitialized = false;
            return true;
        }
    }


    private boolean driveBackPID(double distance, double maxPow)
    {
        if(!drivePIDinitialized)
        {
            driveStartX = robot.getX();
            driveStartY = robot.getY();
            driveDistPID.reset();
            drivePIDinitialized = true;
        }


        double traveled = Math.hypot(robot.getX() - driveStartX, robot.getY() - driveStartY);


        double error = distance - traveled;
        driveDistPID.setTolerance(1.0);
        driveDistPID.setInputRange(0,144);
        driveDistPID.setOutputRange(-1.0,1.0);
        driveDistPID.setSetpoint(0);
        driveDistPID.setInput(-error);


        double power = driveDistPID.performPID();


        if(driveDistPID.onTarget())
        {
            robot.mecanumDrive(0,0,0);
            drivePIDinitialized = false;
            return true;
        }


        robot.mecanumDrive(power,0,0);
        return false;
    }
    private boolean driveForward(double distance, double power)
    {
        if(!driveInitialized)
        {
            backStartX = robot.getX();
            backStartY = robot.getY();
            driveInitialized = true;
        }


        double traveled = Math.hypot(robot.getX() - backStartX, robot.getY() - backStartY);


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
            strafeStartX = robot.getX();
            strafeStartY = robot.getY();
            strafeInitialized = true;
        }


        double traveled = Math.hypot(robot.getX() - strafeStartX, robot.getY() - strafeStartY);


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
            strafeStartX = robot.getX();
            strafeStartY = robot.getY();
            strafeInitialized = true;
        }


        double traveled = Math.hypot(robot.getX() - strafeStartX, robot.getY() - strafeStartY);


        if (traveled < distance) {
            robot.mecanumDrive(0, power, 0); // strafe left
            return false;
        } else {
            robot.mecanumDrive(0, 0, 0);
            strafeInitialized = false;
            return true;
        }
    }

    public boolean shootingSequenceAuto() {
        sequence(true);
        return !sequenceRunning;
    }


    // good
    public void sequence(boolean start) {
        if (start && !sequenceRunning) {
            sequenceRunning = true;
            timer1.reset();
            shootStep = 0;
            flywheelOn = true;
            robot.setFlywheelVelocity(1300);
        }


        if (!sequenceRunning) return;


        double time = timer1.seconds();
        double PRELOAD_TIME = 0.5;
        double SERVO_UP_TIME = 0.2;
        double SERVO_DOWN_TIME = 0.4;
        double FEED_TIME = 0.4;
        double SHOT_TIME = SERVO_UP_TIME + SERVO_DOWN_TIME + FEED_TIME;


        if (time < PRELOAD_TIME) {
            robot.setIntakePower(0.4);
            robot.setPusherDown();
            return;
        }


        double shotTime = time - PRELOAD_TIME;
        double singleShotTime = shotTime - (shootStep * SHOT_TIME);


        if (shootStep < 3) {
            boolean keepIntakeOn = shootStep == 1;
            if (singleShotTime < SERVO_UP_TIME) {
                robot.setPusherUp();
                robot.setIntakePower(keepIntakeOn ? 1.0 : 0.0);// was 0.8
            } else if (singleShotTime < SERVO_UP_TIME + SERVO_DOWN_TIME) {
                robot.setPusherDown();
                robot.setIntakePower(keepIntakeOn ? 0.3 : 0.0);//was0.1
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
        robot.setFlywheelVelocity(0.0);
        sequenceRunning = false;
        flywheelOn = false;
        shootInitialized = false;
    }


    // good
    public void stop() {
        robot.mecanumDrive(0, 0, 0);
        robot.setIntakePower(0);
        robot.setFlywheelVelocity(0);
    }


    private double wrapAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }



    private void sendDashboardTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Auton State", autonState.toString());
        packet.put("Robot X", robot.getX());
        packet.put("Robot Y", robot.getY());
        packet.put("Heading", Math.toDegrees(robot.getHeading()));
        packet.put("error", Math.toDegrees(error));
        packet.put("correction", Math.toDegrees(correction));
        packet.put("Cycle Time (ms)", cycleTimeMs);
        packet.put("Loop Frequency (Hz)", 1000.0 / cycleTimeMs);
        packet.put("motorRBpower", robot.getRightBackPower());


        dashboard.sendTelemetryPacket(packet);
    }


}
