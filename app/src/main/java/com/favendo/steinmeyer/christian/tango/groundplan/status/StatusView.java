package com.favendo.steinmeyer.christian.tango.groundplan.status;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.View;

import java.util.ArrayList;
import java.util.Collection;

/**
 * Custom view that displays the current state of scanning for planes.
 *
 * Created by Christian Steinmeyer on 27.07.2016.
 */
public class StatusView extends View {
    private static final int RADIUS = 20;

    private Paint mWallPaint;
    private Paint mNotWallPaint;
    private Collection<Circle> mCircles = new ArrayList<>();

    public StatusView(Context context) {
        super(context);
        mWallPaint = new Paint();
        mWallPaint.setStyle(Paint.Style.FILL);
        mWallPaint.setStrokeWidth(3);
        mWallPaint.setColor(Color.GREEN);

        mNotWallPaint = new Paint();
        mNotWallPaint.setStyle(Paint.Style.FILL);
        mNotWallPaint.setStrokeWidth(3);
        mNotWallPaint.setColor(Color.RED);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        for (Circle circle : mCircles) {
            canvas.drawCircle(circle.mX, circle.mY, circle.mRadius, circle.mPaint);
        }
    }

    public void clear() {
        mCircles.clear();
    }

    public void addCircle(float x, float y, boolean inWall) {
        Paint paint = inWall ? mWallPaint : mNotWallPaint;
        int width = getWidth();
        int height = getHeight();
        mCircles.add(new Circle(width * x, height * y, RADIUS, paint));
    }
}
