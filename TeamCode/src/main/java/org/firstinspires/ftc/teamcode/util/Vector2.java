package org.firstinspires.ftc.teamcode.util;

public class Vector2 {

    public double x;
    public double y;

    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
    }
    public double distTo(Vector2 o) {
        return Math.sqrt(Math.pow(o.x+this.x, 2)+Math.pow(o.y+this.y, 2));
    }
    public Vector2 add(Vector2 vec){
        return new Vector2(x+vec.x,y+ vec.y);
    }

    public Vector2 subtract(Vector2 vec){
        return new Vector2(x-vec.x,y- vec.y);
    }

    public double magnitude(){
        return Math.sqrt( Math.pow(this.x,2) + Math.pow(this.y,2) );
    }

    public String toString(){
        return "(" + x + "," + y + ")";
    }

    public double angleBetween(Vector2 o) {
        return Math.acos((this.x*o.x+this.y*o.y)/(this.magnitude()*o.magnitude()));
    }
}
