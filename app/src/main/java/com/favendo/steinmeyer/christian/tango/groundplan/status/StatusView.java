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
 * Created by Christian Steinmeyer on 25.07.2016.
 */
public class StatusView extends View {
    private Paint mCandidatePaint;
    private Paint mNoCandidatePaint;
    private int RADIUS = 15;
    private int ALPHA = 50;
    private Collection<Circle> circles = new ArrayList<>();

    public StatusView(Context context) {
        super(context);
        mCandidatePaint = new Paint();
        mCandidatePaint.setStyle(Paint.Style.FILL);
        mCandidatePaint.setStrokeWidth(3);
        mCandidatePaint.setColor(Color.GREEN);
        mCandidatePaint.setAlpha(ALPHA);

        mNoCandidatePaint = new Paint();
        mNoCandidatePaint.setStyle(Paint.Style.FILL);
        mNoCandidatePaint.setStrokeWidth(3);
        mNoCandidatePaint.setColor(Color.RED);
        mNoCandidatePaint.setAlpha(ALPHA);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        synchronized (this) {
            for (Circle circle : circles) {
                canvas.drawCircle(circle.x, circle.y, circle.radius, circle.paint);
            }
        }
    }

    public void clear() {
        circles.clear();
    }

    public void addCircle(float x, float y, boolean isCandidate) {
        Paint paint = isCandidate ? mCandidatePaint : mNoCandidatePaint;
        int width = getWidth();
        int height = getHeight();
        synchronized (this) {
            circles.add(new Circle(width * x, height * y, RADIUS, paint));
        }
    }
}
