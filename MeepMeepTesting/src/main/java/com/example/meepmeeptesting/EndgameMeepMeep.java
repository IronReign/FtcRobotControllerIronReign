package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepTesting.P2D;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.ArrayList;


enum Position {
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

public class EndgameMeepMeep {

    public static double FIELD_INCHES_PER_GRID = 23.5;
    static Field field;

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);
        field = new Field();
        field.isRed = true;
        field.finalizeField();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(16.5, 16.5)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        boolean red = true;

        Pose2d p = Position.START_LEFT_BLUE.getPose();
        Action bot1Action;
        ArrayList<SubZone> arr = field.getSubZones(p);
        if(arr.contains(SubZone.BACKDROP)) {
            bot1Action = myBot.getDrive().actionBuilder(p)
                    .strafeToLinearHeading(new Vector2d(12, red ? -35 : 35), Math.toRadians(180))
                    .waitSeconds(2)
                    .strafeToLinearHeading(new Vector2d(-16, red ? -35 : 35), Math.toRadians(180))
                    .waitSeconds(2)
                    .build();
        }
        else if(arr.contains(SubZone.WING)) {
            bot1Action = myBot.getDrive().actionBuilder(p)
                    .setReversed(true)
                    .splineTo(new Vector2d(1*FIELD_INCHES_PER_GRID, -.5*FIELD_INCHES_PER_GRID), 0)
                    .strafeToLinearHeading(new Vector2d(1*FIELD_INCHES_PER_GRID, -.5*FIELD_INCHES_PER_GRID),  Math.toRadians(180))
                    .setReversed(false)
                    .splineTo(new Vector2d(12, red ? -35 : 35), Math.toRadians(180))
                    .waitSeconds(2)
                    .strafeToLinearHeading(new Vector2d(-16, red ? -35 : 35), Math.toRadians(180))
                    .waitSeconds(2)
                    .build();
        }
        else {
            if(field.getZone(p) == Field.Zone.AUDIENCE) {
                bot1Action = myBot.getDrive().actionBuilder(p)
                        .setReversed(true)
                        .splineTo(new Vector2d(-1.5 * FIELD_INCHES_PER_GRID, -2.5 * FIELD_INCHES_PER_GRID), 0)
                        .splineTo(new Vector2d(1 * FIELD_INCHES_PER_GRID, -2.5 * FIELD_INCHES_PER_GRID), 0)
                        .setReversed(false)
                        .strafeToLinearHeading(new Vector2d(12, red ? -35 : 35), Math.toRadians(180))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(-16, red ? -35 : 35), Math.toRadians(180))
                        .waitSeconds(2)
                        .build();
            }
            else if(field.getZone(p) == Field.Zone.BACKSTAGE){
                bot1Action = myBot.getDrive().actionBuilder(p)
                        .setReversed(true)
                        .splineTo(new Vector2d(-1 * FIELD_INCHES_PER_GRID, -.5 * FIELD_INCHES_PER_GRID), 0)
                        .splineTo(new Vector2d(1 * FIELD_INCHES_PER_GRID, -.5 * FIELD_INCHES_PER_GRID), 0)
                        .strafeToLinearHeading(new Vector2d(1 * FIELD_INCHES_PER_GRID, -.5 * FIELD_INCHES_PER_GRID), Math.toRadians(180))
                        .setReversed(false)
                        .strafeToLinearHeading(new Vector2d(12, red ? -35 : 35), Math.toRadians(180))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(-16, red ? -35 : 35), Math.toRadians(180))
                        .waitSeconds(2)
                        .build();
            }
            else {
                bot1Action = myBot.getDrive().actionBuilder(p)
                        .strafeToLinearHeading(new Vector2d(12, red ? -35 : 35), Math.toRadians(180))
                        .waitSeconds(2)
                        .strafeToLinearHeading(new Vector2d(-16, red ? -35 : 35), Math.toRadians(180))
                        .waitSeconds(2)
                        .build();
            }
        }
        myBot.runAction(bot1Action);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot);
        meepMeep.start();
    }
}