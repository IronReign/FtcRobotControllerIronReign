package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {Alliance alliance = Alliance.RED;


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

    static Pose2d blueRightStageOnePosition;
    static Pose2d blueRightStageTwoPosition;

    static Pose2d blueLeftStageOnePosition;
    static Pose2d blueLeftStageTwoPosition;

    static Pose2d redRightStageOnePosition;
    static Pose2d redRightStageTwoPosition;

    static Pose2d redLeftStageOnePosition;
    static Pose2d redLeftStageTwoPosition;

    static Pose2d purpleEndPosition;
    static Pose2d backdropApproachPosition;
    static Pose2d stageOne, stageTwo;

    static Pose2d aprilTagApproachPosition;
    static Pose2d parkPosition;

    static Pose2d audienceIntermediate = P2D(1,-.5,-10);

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);
        Position startingPosition;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(90), Math.toRadians(90), 15)
                .build();

        startingPosition = Position.START_RIGHT_RED;
        Pose2d p = startingPosition.getPose();

        updateIndexOffsets(startingPosition, 2);
        build(startingPosition);
        Pose2d audienceIntermediate = P2D(1,.5*allianceDirection,-10); //todo move

        System.out.println(Position.START_LEFT_RED.getPose());

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
//                .lineToX(30)
//                .turn(Math.toRadians(90))
//                .lineToY(30)
//                .turn(Math.toRadians(90))
//                .lineToX(0)
//                .turn(Math.toRadians(90))
//                .lineToY(0)
//                .turn(Math.toRadians(90))
//                .build());
        Action bot1;
        bot1 =
        myBot.getDrive().actionBuilder(p)

                .setReversed(true)
                .splineTo(redLeftStageTwoPosition.position, Math.toRadians(100))
                .waitSeconds(1) //eject purple here
                .turnTo(STANDARD_HEADING_RAD)
                .setReversed(true)
                .splineTo(audienceIntermediate.position,audienceIntermediate.heading)
                .splineTo(aprilTagApproachPosition.position, 0)
//                redLeftStageOne

                //starting pose has a heading of -90 degrees
//                .turnTo(Math.toRadians(-45-90)) //no glitch
//                .turnTo(Math.toRadians(-90)) //no glitch
//                .turnTo(-2.443460952792061) //glitch
//                .turnTo(Math.toRadians(270)) //glitch
//                .turnTo(Math.toRadians(-45)) //glitch
//                .turnTo(Math.toRadians(90)) //no glitch
//                .turnTo(Math.toRadians(45)) //glitch
//                .turnTo(Math.toRadians(90)) //no glitch
//                .turnTo(Math.toRadians(135)) //glitch - but 134 in this line doesn't glitch
//                .lineToYLinearHeading(stageOne.position.y, stageOne.heading)

//
//                .strafeTo(stageTwo.position)
//                .turnTo(stageTwo.heading.log())
//                .waitSeconds(1)
//                .setReversed(true)
//
////                //findStandardPosition (for audience side starting positions) comment out otherwise
////                .turnTo(purpleEndPosition.heading)
////                .waitSeconds(1)
////                .strafeTo(purpleEndPosition.position)
//
//                //approachBackdrop
//                .splineToLinearHeading(aprilTagApproachPosition, STANDARD_HEADING)

                .build();
        System.out.println(redLeftStageTwoPosition.heading.log());

        myBot.runAction(bot1);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static int targetIndex = 1;
    public static int targetAprilTagIndex = 1;
    public static int visionProviderIndex;
    public static double aprilTagOffset = .25;
    public static int allianceDirection = -1;

    static double STANDARD_HEADING = 180;
    static double STANDARD_HEADING_RAD = Math.PI;
    Pose2d actionEndPose = new Pose2d(0, 0, 0);
    private Action
            redLeftStageOne, redLeftStageTwo,
            blueLeftStageOne, blueLeftStageTwo,
            blueRightStageOne, blueRightStageTwo,
            redRightStageOne, redRightStageTwo,
            findStandardPosition, approachBackdrop,
            approachAprilTag, findStandardHeading,
            findAprilTagHeading, park;
    ;

    private Action stageOneToRun, stageTwoToRun;
    public static double FIELD_INCHES_PER_GRID = 23.5;
    /**
     * Returns a Pose2d built with inches and radians given field units and degree-based heading
     */
    public static Pose2d P2D (double x, double y, double deg) {
        return new Pose2d(x * FIELD_INCHES_PER_GRID, y * FIELD_INCHES_PER_GRID, Math.toRadians(deg));
    }

    public static void build(Position start){
        Pose2d pose = start.getPose();

        parkPosition = P2D(1.6, allianceDirection * 2.1, STANDARD_HEADING);

        blueRightStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, STAGE_ONE_HEADING);
        blueRightStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, STAGE_TWO_HEADING);

        blueLeftStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, STAGE_ONE_HEADING);
        blueLeftStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, STAGE_TWO_HEADING);

        redLeftStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, -STAGE_ONE_HEADING);
        redLeftStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE + indexStrafeOffset, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, -STAGE_TWO_HEADING);

        redRightStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, -STAGE_ONE_HEADING);
        redRightStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, -STAGE_TWO_HEADING);

        purpleEndPosition = P2D(start.getPose().position.x / FIELD_INCHES_PER_GRID, allianceDirection * .35 / FIELD_INCHES_PER_GRID, STANDARD_HEADING);
        backdropApproachPosition = P2D(1.65, allianceDirection * .35, STANDARD_HEADING);
        aprilTagApproachPosition = P2D(1.8, allianceDirection * 1 + ((targetAprilTagIndex - 3) * aprilTagOffset * allianceDirection ), STANDARD_HEADING);

        if(start.equals(Position.START_RIGHT_RED)) {
            stageOne = redRightStageOnePosition;
            stageTwo = redRightStageTwoPosition;
        }
        if(start.equals(Position.START_LEFT_RED)) {
            stageOne = redLeftStageOnePosition;
            stageTwo = redLeftStageTwoPosition;
        }
        if(start.equals(Position.START_RIGHT_BLUE)) {
            stageOne = blueRightStageOnePosition;
            stageTwo = blueRightStageTwoPosition;
        }
        if(start.equals(Position.START_LEFT_BLUE)) {
            stageOne = blueLeftStageOnePosition;
            stageTwo = blueLeftStageTwoPosition;
        }
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



    public static int INWARD_SCORING_ANGLE = -45;
    public static int MIDDLE_SCORING_ANGLE = -50;
    public static int OUTWARD_SCORING_ANGLE = 30;
    public static double indexStrafeOffset = 0;

    //BASED ON BLUE_RIGHT_START
    public static double STAGE_ONE_Y_COORDINATE = .4;

    public static double STAGE_TWO_X_COORDINATE = -2.1;
    public static double STAGE_TWO_HEADING = 40;
    public static double STAGE_ONE_HEADING = 90;
    public static double BACKSTAGE_X_POSITION_OFFSET = 2.5;

    public static void updateIndexOffsets(Position startingPosition, int randomizer) { // 1, 2 or 3 for randomized prop
        targetIndex = randomizer;
        allianceDirection = startingPosition.getMod()? -1 : 1;

        if(startingPosition.equals(Position.START_RIGHT_BLUE)) {
            stageOne = blueRightStageOnePosition;
            stageTwo = blueRightStageTwoPosition;
            targetAprilTagIndex = targetIndex;
            if (targetIndex == 3) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = 0.65;
                STAGE_TWO_X_COORDINATE = -1.65;
                STAGE_TWO_HEADING = 90 + INWARD_SCORING_ANGLE;
            }

            if (targetIndex == 2) {
                STAGE_ONE_HEADING = 90;

                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_HEADING = 90 + MIDDLE_SCORING_ANGLE;
                STAGE_TWO_X_COORDINATE = -2.1;
            }

            if (targetIndex == 1) {


                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_X_COORDINATE = -1.55;
                STAGE_TWO_HEADING = 90 + OUTWARD_SCORING_ANGLE;
            }
        }

        if(startingPosition.equals(Position.START_LEFT_BLUE)) {
            stageOne = blueLeftStageOnePosition;
            stageTwo = blueLeftStageTwoPosition;
            targetAprilTagIndex = targetIndex;
            if (targetIndex == 1) {
                STAGE_ONE_HEADING = 90;

                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_X_COORDINATE = 0.55;
                STAGE_TWO_HEADING = 90 - OUTWARD_SCORING_ANGLE;
            }

            if (targetIndex == 2) {
                STAGE_ONE_HEADING = 90;

                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_Y_COORDINATE = .5;
                STAGE_TWO_HEADING = 90 - MIDDLE_SCORING_ANGLE;
                STAGE_TWO_X_COORDINATE = 1.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_HEADING = 90;

                STAGE_ONE_Y_COORDINATE = 0.65;
                STAGE_TWO_X_COORDINATE = 0.65;
                STAGE_TWO_HEADING = 90 - INWARD_SCORING_ANGLE;
            }
        }

        if(startingPosition.equals(Position.START_LEFT_RED)) {
            stageOne = redLeftStageOnePosition;
            stageTwo = redLeftStageTwoPosition;
            targetAprilTagIndex = targetIndex + 3;
            if (targetIndex == 1) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -0.65;
                STAGE_TWO_X_COORDINATE = -1.65;
                STAGE_TWO_HEADING = (90 + OUTWARD_SCORING_ANGLE);
            }

            if (targetIndex == 2) {
                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_HEADING = (90 + MIDDLE_SCORING_ANGLE);
                STAGE_TWO_X_COORDINATE = -2.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_X_COORDINATE = -1.55;
                STAGE_TWO_HEADING = (90 + INWARD_SCORING_ANGLE);
            }
        }
        if(startingPosition.equals(Position.START_RIGHT_RED)) {
            stageOne = redRightStageOnePosition;
            stageTwo = redRightStageTwoPosition;
            targetAprilTagIndex = targetIndex + 3;
            if (targetIndex == 1) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -0.65;
                STAGE_TWO_X_COORDINATE = 0.65;
                STAGE_TWO_HEADING = (90 - INWARD_SCORING_ANGLE);
            }

            if (targetIndex == 2) {
                //DO NOTHING, THIS IS THE DEFAULT
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_HEADING = (90 - MIDDLE_SCORING_ANGLE);
                STAGE_TWO_X_COORDINATE = 1.1;
            }

            if (targetIndex == 3) {
                STAGE_ONE_HEADING = 90;
                STAGE_ONE_Y_COORDINATE = -.5;
                STAGE_TWO_X_COORDINATE = 0.55;
                STAGE_TWO_HEADING = (90 - OUTWARD_SCORING_ANGLE);
            }
        }


        System.out.println("index: " + targetAprilTagIndex);
    }

}