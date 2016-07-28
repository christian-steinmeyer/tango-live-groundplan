package com.favendo.steinmeyer.christian.tango.groundplan.status;

import android.graphics.Paint;

/**
 * A simple circle used in the status view.
 *
 * Created by Christian Steinmeyer on 25.07.2016.
 */
class Circle {
    public float mX;
    public float mY;
    public int mRadius;
    public Paint mPaint;

    public Circle(float x, float y, int radius, Paint paint) {
        this.mX = x;
        this.mY = y;
        this.mRadius = radius;
        this.mPaint = paint;
    }
}
