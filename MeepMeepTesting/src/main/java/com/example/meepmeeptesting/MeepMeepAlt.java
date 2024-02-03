package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.entity.Entity;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.ArrayList;


public class MeepMeepAlt {



    public enum Position {
        ORIGIN_DEFAULT (new Pose2d(0, 0, 0)), //this if used will reset the origin to FTC Dashboard's default
        START_LEFT_RED(P2D(-1.5, -2.6, 90)),
        START_RIGHT_RED(P2D(.5, -2.6, 90)),
        START_RIGHT_BLUE(P2D(-1.5, 2.6, -90)),
        START_LEFT_BLUE(P2D(.5, 2.6, -90));
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

    public static final double CENTROID_TO_PIXEL_DISTANCE = 18;

    static ArrayList<Vector2d> purplePixelLocations = new ArrayList<Vector2d>();

    static Pose2d aprilTagApproachPosition;
    static Pose2d audienceIntermediate, pixelStackAudienceIntermediate, pixelStackAudienceIntermediateForward;
    static Pose2d audienceIntermediateForward, audienceIntermediateDeep, driverSidePrep, driverSidePrepForward;
    static Pose2d aprilTagAlign, aprilTagAlignClose,  aprilTagAlignCrossed;
    public static Pose2d pixelStack;



    //values to actually use
    static Pose2d[][] autonPaths;

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);
        Position startingPosition;
        boolean driverSide = true;
        autonPaths = new Pose2d[7][13];
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.5, 16.5)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        RoadRunnerBotEntity myBotPixel = new DefaultBotBuilder(meepMeep)
                .setDimensions(3, 3)
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
        selectedPath = setPath(startingPosition, randomizer, driverSide);
        System.out.println("selected path:\t" + selectedPath);
        System.out.println("audience intermediate:\t" + audienceIntermediate);
        System.out.println(autonPaths[2][4]);

        Action bot1Action, bot2Action, bot3Action, bot4Action;
        if(driverSide) {
            bot1Action =
                    myBot.getDrive().actionBuilder(p)

                            //step 1 - go to purple eject location
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                            .waitSeconds(1) //not needed in real path
                            //step 2 - sweep team prop = lower intake, rotate and eject
                            //for meepmeep this is just the rotation
                            //step 3 - turn to get ready for next spline
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                            //steps 4 & 5 - travel through 1 or 2 final waypoints and then a final correction
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            //steps 6, 7, 8 - run to pixelstack
                            .setReversed(false)
                            .splineTo(switchSides(autonPaths[selectedPath][7].position), switchSides(autonPaths[selectedPath][7].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][8].position), switchSides(autonPaths[selectedPath][8].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                            .waitSeconds(1)
                            //steps 9, 10, 11 - return to backdrop
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][10].position), switchSides(autonPaths[selectedPath][10].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][11].position), switchSides(autonPaths[selectedPath][11].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][12].position), switchSides(autonPaths[selectedPath][12].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(switchSides(autonPaths[selectedPath][6].position).x, switchSides(autonPaths[selectedPath][6].position).y + 20 * allianceDirection * (selectedPath > 3 ? 1 : -1)))
                            .build();
        }
        else {
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
                            //steps 5 & 6 - travel through 1 or 2 final waypoints and then a final correction
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            //steps 8, 9 - run to pixelstack
                            .setReversed(false)
                            .splineTo(switchSides(autonPaths[selectedPath][8].position), switchSides(autonPaths[selectedPath][8].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                            .waitSeconds(1)
                            //steps 10, 12 - return to backdrop
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][10].position), switchSides(autonPaths[selectedPath][10].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][12].position), switchSides(autonPaths[selectedPath][12].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(switchSides(autonPaths[selectedPath][6].position).x, switchSides(autonPaths[selectedPath][6].position).y + 20 * allianceDirection * (selectedPath > 3 ? 1 : -1)))
                            .build();
        }
        //bot2 - builder identical to bot1 - when adjusting the build, do it for bot1 and copy to the other bots
        startingPosition = Position.START_RIGHT_BLUE;
        p = startingPosition.getPose();
        selectedPath = setPath(startingPosition, randomizer, driverSide);
        if(driverSide) {
            bot2Action =
                    myBot.getDrive().actionBuilder(p)

                            //step 1 - go to purple eject location
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                            .waitSeconds(1) //not needed in real path
                            //step 2 - sweep team prop = lower intake, rotate and eject
                            //for meepmeep this is just the rotation
                            //step 3 - turn to get ready for next spline
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                            //steps 4 & 5 - travel through 1 or 2 final waypoints and then a final correction
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            //steps 6, 7, 8 - run to pixelstack
                            .setReversed(false)
                            .splineTo(switchSides(autonPaths[selectedPath][7].position), switchSides(autonPaths[selectedPath][7].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][8].position), switchSides(autonPaths[selectedPath][8].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                            .waitSeconds(1)
                            //steps 9, 10, 11 - return to backdrop
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][10].position), switchSides(autonPaths[selectedPath][10].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][11].position), switchSides(autonPaths[selectedPath][11].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][12].position), switchSides(autonPaths[selectedPath][12].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(switchSides(autonPaths[selectedPath][6].position).x, switchSides(autonPaths[selectedPath][6].position).y + 20 * allianceDirection * (selectedPath > 3 ? 1 : -1)))
                            .build();
        }
        else {
            bot2Action =
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
                            //steps 5 & 6 - travel through 1 or 2 final waypoints and then a final correction
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            //steps 8, 9 - run to pixelstack
                            .setReversed(false)
                            .splineTo(switchSides(autonPaths[selectedPath][8].position), switchSides(autonPaths[selectedPath][8].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                            .waitSeconds(1)
                            //steps 10, 12 - return to backdrop
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][10].position), switchSides(autonPaths[selectedPath][10].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][12].position), switchSides(autonPaths[selectedPath][12].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(switchSides(autonPaths[selectedPath][6].position).x, switchSides(autonPaths[selectedPath][6].position).y + 20 * allianceDirection * (selectedPath > 3 ? 1 : -1)))
                            .build();
        }

        startingPosition = Position.START_LEFT_BLUE;
        p = startingPosition.getPose();
        selectedPath = setPath(startingPosition, randomizer, driverSide);
        if(driverSide) {
            bot3Action =
                    myBot.getDrive().actionBuilder(p)

                            //step 1 - go to purple eject location
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                            .waitSeconds(1) //not needed in real path
                            //step 2 - sweep team prop = lower intake, rotate and eject
                            //for meepmeep this is just the rotation
                            //step 3 - turn to get ready for next spline
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                            //steps 4 & 5 - travel through 1 or 2 final waypoints and then a final correction
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            //steps 6, 7, 8 - run to pixelstack
                            .setReversed(false)
                            .splineTo(switchSides(autonPaths[selectedPath][7].position), switchSides(autonPaths[selectedPath][7].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][8].position), switchSides(autonPaths[selectedPath][8].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                            .waitSeconds(1)
                            //steps 9, 10, 11 - return to backdrop
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][10].position), switchSides(autonPaths[selectedPath][10].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][11].position), switchSides(autonPaths[selectedPath][11].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][12].position), switchSides(autonPaths[selectedPath][12].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(switchSides(autonPaths[selectedPath][6].position).x, switchSides(autonPaths[selectedPath][6].position).y + 20 * allianceDirection * (selectedPath > 3 ? 1 : -1)))
                            .build();
        }
        else {
            bot3Action =
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
                            //steps 5 & 6 - travel through 1 or 2 final waypoints and then a final correction
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            //steps 8, 9 - run to pixelstack
                            .setReversed(false)
                            .splineTo(switchSides(autonPaths[selectedPath][8].position), switchSides(autonPaths[selectedPath][8].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                            .waitSeconds(1)
                            //steps 10, 12 - return to backdrop
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][10].position), switchSides(autonPaths[selectedPath][10].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][12].position), switchSides(autonPaths[selectedPath][12].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(switchSides(autonPaths[selectedPath][6].position).x, switchSides(autonPaths[selectedPath][6].position).y + 20 * allianceDirection * (selectedPath > 3 ? 1 : -1)))
                            .build();
        }

        startingPosition = Position.START_RIGHT_RED;
        p = startingPosition.getPose();
        selectedPath = setPath(startingPosition, randomizer, driverSide);
        if(driverSide) {
            bot4Action =
                    myBot.getDrive().actionBuilder(p)

                            //step 1 - go to purple eject location
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][1].position), switchSides(autonPaths[selectedPath][1].heading.log()))
                            .waitSeconds(1) //not needed in real path
                            //step 2 - sweep team prop = lower intake, rotate and eject
                            //for meepmeep this is just the rotation
                            //step 3 - turn to get ready for next spline
                            .strafeToLinearHeading(switchSides(autonPaths[selectedPath][4].position), switchSides(autonPaths[selectedPath][4].heading.log()))
                            //steps 4 & 5 - travel through 1 or 2 final waypoints and then a final correction
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            //steps 6, 7, 8 - run to pixelstack
                            .setReversed(false)
                            .splineTo(switchSides(autonPaths[selectedPath][7].position), switchSides(autonPaths[selectedPath][7].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][8].position), switchSides(autonPaths[selectedPath][8].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                            .waitSeconds(1)
                            //steps 9, 10, 11 - return to backdrop
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][10].position), switchSides(autonPaths[selectedPath][10].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][11].position), switchSides(autonPaths[selectedPath][11].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][12].position), switchSides(autonPaths[selectedPath][12].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(switchSides(autonPaths[selectedPath][6].position).x, switchSides(autonPaths[selectedPath][6].position).y + 20 * allianceDirection * (selectedPath > 3 ? 1 : -1)))
                            .build();
        }
        else {
            bot4Action =
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
                            //steps 5 & 6 - travel through 1 or 2 final waypoints and then a final correction
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][5].position), switchSides(autonPaths[selectedPath][5].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][6].position), switchSides(autonPaths[selectedPath][6].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            //steps 8, 9 - run to pixelstack
                            .setReversed(false)
                            .splineTo(switchSides(autonPaths[selectedPath][8].position), switchSides(autonPaths[selectedPath][8].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][9].position), switchSides(autonPaths[selectedPath][9].heading.log()))
                            .waitSeconds(1)
                            //steps 10, 12 - return to backdrop
                            .setReversed(true)
                            .splineTo(switchSides(autonPaths[selectedPath][10].position), switchSides(autonPaths[selectedPath][10].heading.log()))
                            .splineTo(switchSides(autonPaths[selectedPath][12].position), switchSides(autonPaths[selectedPath][12].heading.log()))
                            .turnTo(P2D(0, 0, STANDARD_HEADING).heading.log())
                            .waitSeconds(1)
                            .strafeTo(new Vector2d(switchSides(autonPaths[selectedPath][6].position).x, switchSides(autonPaths[selectedPath][6].position).y + 20 * allianceDirection * (selectedPath > 3 ? 1 : -1)))
                            .build();
        }


        myBot2.runAction(bot2Action);
        myBot3.runAction(bot3Action);
        myBot4.runAction(bot4Action);
        myBot.runAction(bot1Action);
//


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .addEntity(myBot4);
        for(Vector2d k : purplePixelLocations) {
            meepMeep.addEntity(new Pixel(meepMeep, k, new Vector2d(0, 0)));
        }
        meepMeep.start();
    }
    public static int selectedPath;
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

    public static int setPath(Position startingPosition, int randomizer, boolean driverSide) { // 1, 2 or 3 for randomized prop
        if(randomizer == 0)
            randomizer = 2;
        aprilTagApproachPosition = P2D(1.5,1.5, STANDARD_HEADING);
        audienceIntermediate = P2D(1,.5,-10);
        pixelStackAudienceIntermediate = P2D(driverSide?-1.5:1,driverSide?2.5:.5,-10);
        pixelStackAudienceIntermediateForward = P2D(driverSide?1:1,driverSide?2.5:.3,-10);
        audienceIntermediateForward = P2D(driverSide?-1.5:1.4, driverSide?2.5:.5, STANDARD_HEADING);
        driverSidePrep = P2D(1, 2.5, -10);
        driverSidePrepForward = P2D(1, 2.5, STANDARD_HEADING);
        audienceIntermediateDeep = P2D(driverSide?1:1.5,driverSide?2.5:.5,-10);
        allianceDirection = startingPosition.getMod()? -1 : 1;

        //aprilTagAlign = new Pose2d (new Vector2d(switchSides(aprilTagApproachPosition.position).x,switchSides(aprilTagApproachPosition.position).y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlign = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
//        aprilTagAlignClose = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x-1,aprilTagApproachPosition.position.y + ((targetAprilTagIndex - 2) *-allianceDirection* aprilTagOffset)), 0);
        aprilTagAlign = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)), 0);
        aprilTagAlignClose = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x - 1,aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)),
                0);
        aprilTagAlignCrossed = new Pose2d (new Vector2d(aprilTagApproachPosition.position.x,aprilTagApproachPosition.position.y + ((randomizer - 2) * -allianceDirection * aprilTagOffset)), Math.toRadians(-90));
        pixelStack = P2D(-2.25, driverSide?1.5:.5, STANDARD_HEADING);

        //assemble the paths
        autonPaths[1][1] = P2D(-2, driverSide?1.75:.5, 90);
        autonPaths[1][2] = P2D(0, 0, driverSide?90:-90);
        autonPaths[1][3] = P2D(0, 0, driverSide?-90:STANDARD_HEADING);
        autonPaths[1][4] = P2D(-2, driverSide?2.5:.5, STANDARD_HEADING);
        autonPaths[1][5] = pixelStackAudienceIntermediateForward;
        autonPaths[1][6] = aprilTagAlign;
        autonPaths[1][7] = driverSidePrepForward;
        autonPaths[1][8] = audienceIntermediateForward;
        autonPaths[1][9] = pixelStack;
        autonPaths[1][10] = pixelStackAudienceIntermediate;
        autonPaths[1][11] = driverSidePrep;
        autonPaths[1][12] = aprilTagAlign;

        autonPaths[2][1] = P2D(driverSide?-1.5:-1.2, driverSide?1.8:.43, 90);
        autonPaths[2][2] = P2D(0, 0, driverSide?90:-90-35);
        autonPaths[2][3] = P2D(0, 0, driverSide?-90:STANDARD_HEADING);
        autonPaths[2][4] = P2D(-2, 2.5, STANDARD_HEADING);
        autonPaths[2][5] = pixelStackAudienceIntermediateForward;
        autonPaths[2][6] = aprilTagAlign;
        autonPaths[2][7] = driverSidePrepForward;
        autonPaths[2][8] = audienceIntermediateForward;
        autonPaths[2][9] = pixelStack;
        autonPaths[2][10] = pixelStackAudienceIntermediate;
        autonPaths[2][11] = driverSidePrep;
        autonPaths[2][12] = aprilTagAlign;
        System.out.println(switchSides(aprilTagAlign.position).y);

        autonPaths[3][1] = P2D(-1.6, driverSide?1.75:1, driverSide?45:90);
        autonPaths[3][2] = P2D(0, 0, driverSide?45:-30);
        autonPaths[3][3] = P2D(0, 0, -130);
        autonPaths[3][4] = P2D(-2, 2.5, STANDARD_HEADING);
        autonPaths[3][5] = audienceIntermediateDeep;
        autonPaths[3][6] = driverSide?aprilTagAlign:aprilTagAlignCrossed;
        autonPaths[3][7] = driverSidePrepForward;
        autonPaths[3][8] = audienceIntermediateForward;
        autonPaths[3][9] = pixelStack;
        autonPaths[3][10] = pixelStackAudienceIntermediate;
        autonPaths[3][11] = driverSidePrep;
        autonPaths[3][12] = aprilTagAlign;

        autonPaths[4][1] = P2D(driverSide?.8:(startingPosition.getPose().position.x/FIELD_INCHES_PER_GRID+.3), driverSide?1.3:1.25, driverSide?180:90);
        autonPaths[4][2] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[4][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[4][4] = P2D(.81, 1.31, 180);
        autonPaths[4][5] = aprilTagAlignClose;
        autonPaths[4][6] = aprilTagAlign;
        autonPaths[4][7] = driverSidePrepForward;
        autonPaths[4][8] = audienceIntermediateForward;
        autonPaths[4][9] = pixelStack;
        autonPaths[4][10] = pixelStackAudienceIntermediate;
        autonPaths[4][11] = driverSidePrep;
        autonPaths[4][12] = aprilTagAlign;

        autonPaths[5][1] = P2D(driverSide?.5:startingPosition.getPose().position.x/FIELD_INCHES_PER_GRID, 1.8, 90);
        autonPaths[5][2] = P2D(0, 0, 90);
        autonPaths[5][3] = P2D(0, 0, 170);
        autonPaths[5][4] = P2D(.51, 1.81, 170);
        autonPaths[5][5] = aprilTagAlignClose;
        autonPaths[5][6] = aprilTagAlign;
        autonPaths[5][7] = driverSidePrepForward;
        autonPaths[5][8] = audienceIntermediateForward;
        autonPaths[5][9] = pixelStack;
        autonPaths[5][10] = pixelStackAudienceIntermediate;
        autonPaths[5][11] = driverSidePrep;
        autonPaths[5][12] = aprilTagAlign;

        autonPaths[6][1] = P2D(driverSide?.4:1.4, driverSide?1.75:41.2 / 23.5 , driverSide?45:-15);
        autonPaths[6][2] = P2D(0, 0, driverSide?45:125);
        autonPaths[6][3] = P2D(0, 0, STANDARD_HEADING);
        autonPaths[6][4] = P2D(.51, 1.76, STANDARD_HEADING);
        autonPaths[6][5] = aprilTagAlignClose;
        autonPaths[6][6] = aprilTagAlign;
        autonPaths[6][7] = driverSidePrepForward;
        autonPaths[6][8] = audienceIntermediateForward;
        autonPaths[6][9] = pixelStack;
        autonPaths[6][10] = pixelStackAudienceIntermediate;
        autonPaths[6][11] = driverSidePrep;
        autonPaths[6][12] = aprilTagAlign;

        int rando = randomizer;
        if (allianceDirection==1 && randomizer==1) rando = 3;
        if (allianceDirection==1 && randomizer==3) rando = 1;
        int ret =  (startingPosition.equals(Position.START_RIGHT_RED)||startingPosition.equals(Position.START_LEFT_BLUE))?
                3+rando :rando;
        purplePixelLocations.add(new Vector2d(switchSides(autonPaths[ret][1].position).x + CENTROID_TO_PIXEL_DISTANCE*Math.cos(switchSides(autonPaths[ret][2].heading.log())), switchSides(autonPaths[ret][1].position).y + CENTROID_TO_PIXEL_DISTANCE*Math.sin(switchSides(autonPaths[ret][2].heading.log()))));
        return ret;
    }

}
