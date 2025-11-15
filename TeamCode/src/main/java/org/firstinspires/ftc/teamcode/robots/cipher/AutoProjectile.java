package org.firstinspires.ftc.teamcode.robots.cipher;

public class AutoProjectile {
    /*
    - Get distance from distance sensor OR function that calculates distance based on position on virtual field?
    - Input angular velocity + angle into flywheel controlling function
    - PID to maintain velocities and angles
     */
    public class TrajectoryCalculator {
        private double distance;
        private double angularVelocity;
        private double theta;
        //TODO: Update
        public int GOAL_HEIGHT = 54;
        public int ROBOT_HEIGHT = 10;
        public int ENCODER_TICKS_PER_REVOLUTION = 20;
        public int GEAR_RATIO = 3;
        public double GRAVITY = 9.8;
        public double FLYWHEEL_RADIUS;


        public TrajectoryCalculator (double distance){
            this.distance = distance;
        }

        public TrajectorySolution getTrajectorySolution() {
            double travelHeight = GOAL_HEIGHT - ROBOT_HEIGHT;
            double flightTime = Math.sqrt((2 * travelHeight) / GRAVITY);

            double horizontalVelocity = distance / flightTime;
            double verticalVelocity = GRAVITY * flightTime;
            double velocity = Math.sqrt(Math.pow(horizontalVelocity, 2)) + Math.pow(verticalVelocity, 2);

            double angularVelocity = velocity / FLYWHEEL_RADIUS;
            angularVelocity *= ((ENCODER_TICKS_PER_REVOLUTION * GEAR_RATIO)/(2*Math.PI));

            double theta = Math.asin((GRAVITY*flightTime)/velocity);
            return new TrajectorySolution(angularVelocity, theta);
        }
    }

    public class TrajectorySolution {
        private double angularVelocity;
        private double theta;

        public TrajectorySolution(double angularVelocity, double theta){
            this.angularVelocity = angularVelocity;
            this.theta = theta;
        }

        public double getAngularVelocity() {
            return angularVelocity;
        }

        public double getTheta(){
            return theta;
        }
    }
}


