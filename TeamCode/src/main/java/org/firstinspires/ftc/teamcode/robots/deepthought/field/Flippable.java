package org.firstinspires.ftc.teamcode.robots.deepthought.field;

public abstract class Flippable {
    public double x, y;
    public double heading;
    public void flip(boolean isRed) {
        //CREATE YOUR IMPLEMENTATION OF FLIP FOR EACH SEASON
        //DEFINE FLIPPABLES AS RED
        if(!isRed) {
            x = -x;
            y = -y;
            heading = heading - 180;
        }
    }
}
