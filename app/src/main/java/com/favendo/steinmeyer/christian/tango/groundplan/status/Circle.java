package com.favendo.steinmeyer.christian.tango.groundplan.status;

import android.graphics.Paint;

/**
 * A simple circle used in the status view.
 *
 * Created by Christian Steinmeyer on 27.07.2016.
 */
public class Circle {
    public float x;
    public float y;
    public int radius;
    public Paint paint;

    public Circle(float x, float y, int radius, Paint paint) {
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.paint = paint;
    }
}
