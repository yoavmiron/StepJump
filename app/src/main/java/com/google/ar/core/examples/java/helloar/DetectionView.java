package com.google.ar.core.examples.java.helloar;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.view.View;

public class DetectionView extends View {
    Paint paint;
    Float x1;
    Float y1;
    Float x2;
    Float y2;

    public DetectionView(Context context) {
        super(context);
        paint = new Paint();
        paint.setStrokeWidth(20);
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(Color.GREEN);
        this.setBackgroundColor(Color.TRANSPARENT);
    }

    public DetectionView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        paint = new Paint();
        paint.setStrokeWidth(20);
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(Color.GREEN);
        this.setBackgroundColor(Color.TRANSPARENT);
    }

    public DetectionView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        paint = new Paint();
        paint.setStrokeWidth(20);
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(Color.GREEN);
        this.setBackgroundColor(Color.TRANSPARENT);
    }

    public DetectionView(Context context, @Nullable AttributeSet attrs, int defStyleAttr, int defStyleRes)
    {
        super(context,attrs,defStyleAttr,defStyleRes);
        paint = new Paint();
        paint.setStrokeWidth(20);
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(Color.GREEN);
        this.setBackgroundColor(Color.TRANSPARENT);
    }


    public void setRect(float left, float top, float right, float bottom) {
        x1 = left;
        y1 = top;
        x2 = right;
        y2 = bottom;

    }


    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (x1 != null) {
            canvas.drawRect(x1, y1, x2, y2, paint);
        } else {
            canvas.drawRect(0, 0, canvas.getWidth(), canvas.getHeight() / 2, paint);
        }
    }
}
