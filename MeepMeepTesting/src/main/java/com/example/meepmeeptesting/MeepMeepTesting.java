package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {

        Alliance alliance = Alliance.RED;
        Position startingPosition = Position.START_LEFT_RED;
        Pose2d blueRightStageOnePosition;
        Pose2d blueRightStageTwoPosition;

        Pose2d blueLeftStageOnePosition;
        Pose2d blueLeftStageTwoPosition;

        Pose2d redRightStageOnePosition;
        Pose2d redRightStageTwoPosition;

        Pose2d redLeftStageOnePosition;
        Pose2d redLeftStageTwoPosition;

        Pose2d purpleEndPosition;
        Pose2d backdropApproachPosition;

        Pose2d aprilTagApproachPosition;
        Pose2d parkPosition;

        Pose2d pose = startingPosition.getPose();

        blueRightStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, STAGE_ONE_HEADING);
        blueRightStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, STAGE_TWO_HEADING);

        blueLeftStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, STAGE_ONE_HEADING);
        blueLeftStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE + BACKSTAGE_X_POSITION_OFFSET - indexStrafeOffset, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, STAGE_TWO_HEADING);

        redLeftStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, -STAGE_ONE_HEADING);
        redLeftStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE + indexStrafeOffset, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, -STAGE_TWO_HEADING);

        redRightStageOnePosition = P2D(pose.position.x / FIELD_INCHES_PER_GRID, STAGE_ONE_Y_COORDINATE, -STAGE_ONE_HEADING);
        redRightStageTwoPosition = P2D(STAGE_TWO_X_COORDINATE, blueRightStageOnePosition.position.y/FIELD_INCHES_PER_GRID, -STAGE_TWO_HEADING);

        purpleEndPosition = P2D(startingPosition.getPose().position.x / FIELD_INCHES_PER_GRID, allianceDirection * .35 * FIELD_INCHES_PER_GRID, STANDARD_HEADING);
        backdropApproachPosition = P2D(1.65, allianceDirection * .35, STANDARD_HEADING);
        aprilTagApproachPosition = P2D(1.8, allianceDirection * 1.5 + (targetAprilTagIndex - 3) * aprilTagOffset, STANDARD_HEADING);

        parkPosition = P2D(1.6, allianceDirection * 2.1, STANDARD_HEADING);

        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(90), Math.toRadians(90), 15)
                .build();



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

        updateIndexOffsets(startingPosition, 1);
        myBot.runAction(myBot.getDrive().actionBuilder(startingPosition.pose)
                //redLeftStageOne
                        .lineToYLinearHeading(redLeftStageOnePosition.position.y, redLeftStageOnePosition.heading)

                //redLeftStageTwo
                        .strafeTo(redLeftStageTwoPosition.position)
                        .turnTo(redLeftStageTwoPosition.heading)

                //findStandardPosition (for audience side starting positions) comment out otherwise
                        .turnTo(purpleEndPosition.heading)
                        .strafeTo(purpleEndPosition.position)

                //approachBackdrop
                .turnTo(purpleEndPosition.heading) //todo, why is this here?
                .strafeTo(backdropApproachPosition.position)

                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static int targetIndex = 1;
    public static int targetAprilTagIndex = 1;
    public static int visionProviderIndex;
    public static double aprilTagOffset = .2;
    public static int allianceDirection = -1;

    static double STANDARD_HEADING = 180;
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
    public enum Position {
        ORIGIN_DEFAULT (new Pose2d(0, 0, 0)), //this if used will reset the origin to FTC Dashboard's default
        START_LEFT_RED(P2D(-1.5, -2.5, -90)),
        START_RIGHT_RED(P2D(.5, -2.5, -90)),
        START_RIGHT_BLUE(P2D(-1.5, 2.5, 90)),
        START_LEFT_BLUE(P2D(.5, 2.5, 90));
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

        if(startingPosition.equals(Position.START_RIGHT_BLUE)) {
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


    }

}