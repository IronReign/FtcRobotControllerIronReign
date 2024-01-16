package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.entity.Entity;
import com.noahbres.meepmeep.core.util.FieldUtil;

import org.jetbrains.annotations.NotNull;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;

public class Pixel implements Entity {
    private String tag = "RING_ENTITY";

    private int zIndex = 10;

    private MeepMeep meepMeep;

    private double canvasWidth = FieldUtil.getCANVAS_WIDTH();
    private double canvasHeight = FieldUtil.getCANVAS_WIDTH();

    private double radius = 2.5;
    private double thickness = 1;

    private Vector2d position;
    private Vector2d velocity;

    public Pixel(MeepMeep meepMeep, Vector2d initialPosition, Vector2d initialVelocity) {
        this.meepMeep = meepMeep;

        this.position = initialPosition;
        this.velocity = initialVelocity;
    }

    @Override
    public void update(long deltaTime) {
        position = position.plus(this.velocity.times(deltaTime / 1e9));

        if (position.x > FieldUtil.getFIELD_WIDTH() / 2.0 || position.x < -FieldUtil.getFIELD_WIDTH() / 2.0 || position.y > FieldUtil.getFIELD_HEIGHT() / 2.0 || position.y < -FieldUtil.getFIELD_HEIGHT() / 2.0) {
            meepMeep.requestToRemoveEntity(this);
        }
    }

    @Override
    public void render(Graphics2D gfx, int i, int i1) {
        Vector2d screenCoords = FieldUtil.fieldCoordsToScreenCoords(position);
        double radPixels = FieldUtil.scaleInchesToPixel(radius, canvasWidth, canvasHeight);

        gfx.setStroke(new BasicStroke((int) FieldUtil.scaleInchesToPixel(thickness, canvasWidth, canvasHeight)));
        gfx.setColor(new Color(134, 34, 222));
        gfx.drawOval(
                (int) (screenCoords.x - radPixels),
                (int) (screenCoords.y - radPixels),
                (int) (radPixels * 2),
                (int) (radPixels * 2)
        );
    }

    @NotNull
    @Override
    public MeepMeep getMeepMeep() {
        return null;
    }

    @NotNull
    @Override
    public String getTag() {
        return tag;
    }

    @Override
    public int getZIndex() {
        return this.zIndex;
    }

    @Override
    public void setZIndex(int i) {
        this.zIndex = i;
    }

    @Override
    public void setCanvasDimensions(double width, double height) {
        this.canvasWidth = width;
        this.canvasHeight = height;
    }
}