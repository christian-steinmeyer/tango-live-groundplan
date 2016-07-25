package com.favendo.steinmeyer.christian.tango.groundplan.floorplan;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.view.View;

/**
 * Custom View that draws the plan in 2D.
 */
public class FloorplanView extends View {

    private Paint mPaint;
    private Floorplan mFloorplan;

    public FloorplanView(Context context, Floorplan floorplan) {
        super(context);
        mPaint = new Paint();
        mPaint.setStrokeWidth(10);
        mPaint.setTextSize(50);
        this.mFloorplan = floorplan;
    }

    @Override
    public void onDraw(Canvas canvas) {
        mFloorplan.drawOnCanvas(canvas, mPaint);
    }
}
