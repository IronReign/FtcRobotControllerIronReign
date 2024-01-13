package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    public enum Position {
        ORIGIN_DEFAULT (new Pose2d(0, 0, 0)), //this if used will reset the origin to FTC Dashboard's default
        START_LEFT_RED(P2D(-1.5, -2.6, -90)),
        START_RIGHT_RED(P2D(.5, -2.6, -90)),
        START_RIGHT_BLUE(P2D(-1.5, 2.6, 90)),
        START_LEFT_BLUE(P2D(.5, 2.6, 90));
        private final Pose2d pose;

        public boolean getMod() {
            return this.name() == START_LEFT_RED.name() || this.name() == START_RIGHT_RED.name() ?  true : false;
        }

        Position(Pose2d pose) {
            this.pose = pose;
        }

        public Pose2d getPose() {
            return pose;
        }
    }

    static Pose2d aprilTagApproachPosition;
    static Pose2d audienceIntermediate;
    static Pose2d aprilTagAlign, aprilTagAlignClose;


    //values to actually use
    static Pose2d[][] autonPaths;

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);
        Position startingPosition;
        autonPaths = new Pose2d[7][6];
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.5, 16.5)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        //extra bots for showing all starting positions
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.5, 16.5)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.5, 16.5)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.5, 16.5)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        int randomizer = 1; //1, 2 or 3
        Pose2d p;

        startingPosition = Position.START_LEFT_RED;
        p = startingPosition.getPose();
        selectedPath = setPath(startingPosition, randomizer);
        System.out.println("selected path:\t" + selectedPath);
        System.out.println("audience intermediate:\t" + audienceIntermediate);
        System.out.println(autonPaths[2][4]);

        Action bot1Action, bot2Action, bot3Action, bot4Action;
        bot1Action =
                myBot.getDrive().actionBuilder(p)

                        .setReversed(true)
                        //step 1 - go to purple eject location
                        .splineTo(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                        .waitSeconds(1) //not needed in real path
                        //step 2 - sweep team prop = lower intake, rotate and eject
                        //for meepmeep this is just the rotation
                        .turnTo(switchSides(autonPaths[selectedPath][2].heading.log()))
                        .waitSeconds(1) //not needed in real path
                        //step 3 - turn to get ready for next spline
                        .turnTo(switchSides(autonPaths[selectedPath][3].heading.log()))
                        //steps 4 & 5 - travel through 1 or 2 final waypoints
                        .setReversed(true)
                        .splineTo(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                        .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))

                        .build();

        //bot2 - builder identical to bot1 - when adjusting the build, do it for bot1 and copy to the other bots
        startingPosition = Position.START_RIGHT_BLUE;
        p = startingPosition.getPose();
        selectedPath = setPath(startingPosition, randomizer);
        bot2Action =
                myBot2.getDrive().actionBuilder(p)
                        .setReversed(true)
                        //step 1 - go to purple eject location
                        .splineTo(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                        .waitSeconds(1) //not needed in real path
                        //step 2 - sweep team prop = lower intake, rotate and eject
                        //for meepmeep this is just the rotation
                        .turnTo(switchSides(autonPaths[selectedPath][2].heading.log()))
                        .waitSeconds(1) //not needed in real path
                        //step 3 - turn to get ready for next spline
                        .turnTo(switchSides(autonPaths[selectedPath][3].heading.log()))
                        //steps 4 & 5 - travel through 1 or 2 final waypoints
                        .setReversed(true)
                        .splineTo(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                        .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))

                        .build();

        startingPosition = Position.START_LEFT_BLUE;
        p = startingPosition.getPose();
        selectedPath = setPath(startingPosition, randomizer);
        bot3Action =
                myBot3.getDrive().actionBuilder(p)

                        .setReversed(true)
                        //step 1 - go to purple eject location
                        .splineTo(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                        .waitSeconds(1) //not needed in real path
                        //step 2 - sweep team prop = lower intake, rotate and eject
                        //for meepmeep this is just the rotation
                        .turnTo(switchSides(autonPaths[selectedPath][2].heading.log()))
                        .waitSeconds(1) //not needed in real path
                        //step 3 - turn to get ready for next spline
                        .turnTo(switchSides(autonPaths[selectedPath][3].heading.log()))
                        //steps 4 & 5 - travel through 1 or 2 final waypoints
                        .setReversed(true)
                        .splineTo(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                        .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))

                        .build();

        startingPosition = Position.START_RIGHT_RED;
        p = startingPosition.getPose();
        selectedPath = setPath(startingPosition, randomizer);
        bot4Action =
                myBot4.getDrive().actionBuilder(p)

                        .setReversed(true)
                        //step 1 - go to purple eject location
                        .splineTo(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                        .waitSeconds(1) //not needed in real path
                        //step 2 - sweep team prop = lower intake, rotate and eject
                        //for meepmeep this is just the rotation
                        .turnTo(switchSides(autonPaths[selectedPath][2].heading.log()))
                        .waitSeconds(1) //not needed in real path
                        //step 3 - turn to get ready for next spline
                        .turnTo(switchSides(autonPaths[selectedPath][3].heading.log()))
                        //steps 4 & 5 - travel through 1 or 2 final waypoints
                        .setReversed(true)
                        .splineTo(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                        .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))

                        .build();

        myBot.runAction(bot1Action);
        myBot2.runAction(bot2Action);
        myBot3.runAction(bot3Action);
        myBot4.runAction(bot4Action);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .addEntity(myBot4)
                .start();
    }
    public static int selectedPath;
    public static int targetIndex = 1;
    public static int targetAprilTagIndex = 1;
    public static int visionProviderIndex;

    public static int allianceDirection = -1;

    static double STANDARD_HEADING = 180;
    static double STANDARD_HEADING_RAD = Math.PI;

    public static double FIELD_INCHES_PER_GRID = 23.5;
    public static double aprilTagOffset = 6.0; //this needs to be in inches
    /**
     * Returns a Pose2d built with inches and radians given field units and degree-based heading
     */
    public static Pose2d P2D (double x, double y, double deg) {
        return new Pose2d(x * FIELD_INCHES_PER_GRID, y * FIELD_INCHES_PER_GRID, Math.toRadians(deg));
    }

    public static Pose2d switchSides(Pose2d p) {
        return new Pose2d(p.position.x, allianceDirection*p.position.y, -allianceDirection*p.heading.log());
    }
    public static Vector2d switchSides(Vector2d v) {
        return new Vector2d(v.x, allianceDirection*v.y);
    }
    public static double switchSides(double r) {
        return -allianceDirection*r;
    }

    public enum Alliance {
        RED(true), BLUE(false);

        private boolean mod;

        Alliance(boolean mod) {
            this.mod = mod;
        }
        public boolean getMod() {
            return mod;
        }
        public void Toggle(){this.mod = !this.mod;}
    }

    public static int setPath(Position startingPosition, int randomizer) { // 1, 2 or 3 for randomized prop
        aprilTagApproachPosition = P2D(1.8,   1.5, STANDARD_HEADING);
        audienceIntermediate = P2D(1,.5,-10);
        allianceDirection = startingPosition.getMod()? -1 : 1;
        //aprilTagAlign = new Pose2d (new Vector2d(switchSides(aprilTagApproachPosition.position).x,switchSides(aprilTagApproachPosition.position).y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlign = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlignClose = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x-1,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
        aprilTagAlign = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((randomizer - 2) *-allianceDirection* aprilTagOffset)), 0);
        aprilTagAlignClose = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x-1,aprilTagApproachPosition.position.y + ((randomizer - 2) *-allianceDirection* aprilTagOffset)), 0);
        System.out.println(aprilTagOffset);
        //assemble the paths
        autonPaths[1][1] = P2D(-2, .5, 90);
        autonPaths[1][2] = P2D(0, 0, -90);
        autonPaths[1][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[1][4] = audienceIntermediate;
        autonPaths[1][5] = aprilTagAlign;

        autonPaths[2][1] = P2D(-1.2, .45, 47);
        autonPaths[2][2] = P2D(0, 0, -90-47);
        autonPaths[2][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[2][4] = audienceIntermediate;
        autonPaths[2][5] = aprilTagAlign;

        autonPaths[3][1] = P2D(-1.65, 1, 90);
        autonPaths[3][2] = P2D(0, 0, -30);
        autonPaths[3][3] = P2D(0, 0, -130);
        autonPaths[3][4] = audienceIntermediate;
        autonPaths[3][5] = aprilTagAlign;

        autonPaths[4][1] = P2D(startingPosition.pose.position.x/FIELD_INCHES_PER_GRID+.15, 1.25, 90);
        autonPaths[4][2] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[4][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[4][4] = aprilTagAlignClose;
        autonPaths[4][5] = aprilTagAlign;

        autonPaths[5][1] = P2D(startingPosition.pose.position.x/FIELD_INCHES_PER_GRID, 1.7, 90);
        autonPaths[5][2] = P2D(0, 0, 90);
        autonPaths[5][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[5][4] = aprilTagAlignClose;
        autonPaths[5][5] = aprilTagAlign;

        autonPaths[6][1] = P2D(1.55, 1.5, -15);
        autonPaths[6][2] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[6][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[6][4] = aprilTagAlignClose;
        autonPaths[6][5] = aprilTagAlign;

        int rando = randomizer;
        if (allianceDirection==1 && randomizer==1) rando = 3;
        if (allianceDirection==1 && randomizer==3) rando = 1;
        return (startingPosition.equals(Position.START_RIGHT_RED)||startingPosition.equals(Position.START_LEFT_BLUE))?
                3+rando :rando;
    }

}