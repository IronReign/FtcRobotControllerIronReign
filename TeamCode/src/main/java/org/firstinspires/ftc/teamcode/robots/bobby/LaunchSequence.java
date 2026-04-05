package org.firstinspires.ftc.teamcode.robots.bobby;
import com.qualcomm.robotcore.util.ElapsedTime;
public class LaunchSequence {
    private final ElapsedTime timer = new ElapsedTime();


    private boolean running = false;
    private int shootStep = 0;


    public void start(Robot robot) {
        running = true;
        timer.reset();
        shootStep = 0;
        robot.shooter.setFlywheelVelocity(125);
    }


    public boolean isRunning() {
        return running;
    }


    public void update(Robot robot) {
        if (!running) {
            return;
        }


        double time = timer.seconds();


        double preloadTime = 0.5;
        double servoUpTime = 0.2;
        double servoDownTime = 0.4;
        double feedTime = 0.4;
        double shotTime = servoUpTime + servoDownTime + feedTime;


        if (time < preloadTime) {
            robot.intake.setIntakePower(0.3);
            robot.shooter.setPusherDown();
            return;
        }


        double elapsedShotTime = time - preloadTime;
        double singleShotTime = elapsedShotTime - (shootStep * shotTime);


        if (shootStep < 3) {
            boolean keepIntakeOn = (shootStep == 1);


            if (singleShotTime < servoUpTime) {
                robot.shooter.setPusherUp();
                robot.intake.setIntakePower(keepIntakeOn ? 0.8 : 0.0);
            } else if (singleShotTime < servoUpTime + servoDownTime) {
                robot.shooter.setPusherDown();
                robot.intake.setIntakePower(keepIntakeOn ? 1.0 : 0.0);
            } else if (singleShotTime < shotTime) {
                robot.intake.setIntakePower(1.0);
                robot.shooter.setPusherDown();
            } else {
                shootStep++;
            }
            return;
        }


        stop(robot);
    }


    public void stop(Robot robot) {
        robot.intake.setIntakePower(0.0);
        robot.shooter.setPusherDown();
        robot.shooter.setFlywheelVelocity(0.0);
        running = false;
        shootStep = 0;
    }
}
