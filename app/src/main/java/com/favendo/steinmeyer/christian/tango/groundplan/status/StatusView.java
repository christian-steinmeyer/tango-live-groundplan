package com.favendo.steinmeyer.christian.tango.groundplan.status;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;
import android.view.View;

import java.util.ArrayList;
import java.util.Collection;

/**
 * Custom view that displays the current state of scanning for planes.
 *
 * Created by Christian Steinmeyer on 27.07.2016.
 */
public class StatusView extends View {
    private String TAG = "StatusView";
    private Paint mWallPaint;
    private Paint mNotWallPaint;
    private int RADIUS = 20;
    private Collection<Circle> circles = new ArrayList<>();

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
        Log.i(TAG, "drawing " + circles.size() + " circles");
        for (Circle circle : circles) {
            canvas.drawCircle(circle.x, circle.y, circle.radius, circle.paint);
        }
    }

    public void clear() {
        Log.i(TAG, "Clearing");
        circles.clear();
    }

    public void addCircle(float x, float y, boolean inWall) {
        Paint paint = inWall ? mWallPaint : mNotWallPaint;
        int width = getWidth();
        int height = getHeight();
        circles.add(new Circle(width * x, height * y, RADIUS, paint));
    }
}
