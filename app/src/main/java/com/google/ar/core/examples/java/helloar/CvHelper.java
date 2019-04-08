package com.google.ar.core.examples.java.helloar;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.view.View;

import java.util.ArrayList;

public class CvHelper extends View {

    private ArrayList<Line> lines;
    private ArrayList<Dot> dots;
    private Paint linePaint;
    private Paint dotPaint;
    private float radius;

    public CvHelper(Context context) {
        super(context);
        linePaint = new Paint();
        linePaint.setStrokeWidth(3);
        linePaint.setStyle(Paint.Style.STROKE);
        linePaint.setColor(Color.BLUE);
        dotPaint = new Paint();
        dotPaint.setStrokeWidth(3);
        dotPaint.setStyle(Paint.Style.STROKE);
        dotPaint.setColor(Color.RED);
        lines = new ArrayList<>();
        dots = new ArrayList<>();
        this.setBackgroundColor(Color.TRANSPARENT);
        radius=3.0f;
    }


    public CvHelper(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        linePaint = new Paint();
        linePaint.setStrokeWidth(3);
        linePaint.setStyle(Paint.Style.STROKE);
        linePaint.setColor(Color.BLUE);
        dotPaint = new Paint();
        dotPaint.setStrokeWidth(3);
        dotPaint.setStyle(Paint.Style.STROKE);
        dotPaint.setColor(Color.RED);
        lines = new ArrayList<>();
        dots = new ArrayList<>();
        this.setBackgroundColor(Color.TRANSPARENT);
        radius=3.0f;
    }

    public CvHelper(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        linePaint = new Paint();
        linePaint.setStrokeWidth(3);
        linePaint.setStyle(Paint.Style.STROKE);
        linePaint.setColor(Color.BLUE);
        dotPaint = new Paint();
        dotPaint.setStrokeWidth(3);
        dotPaint.setStyle(Paint.Style.STROKE);
        dotPaint.setColor(Color.RED);
        lines = new ArrayList<>();
        dots = new ArrayList<>();
        this.setBackgroundColor(Color.TRANSPARENT);
        radius=3.0f;
    }


    public void drawLine(Line line) {
        lines.add(line);
    }

    public void drawDot(Dot dot) {
        dots.add(dot);
    }

    public void cleanUp() {
        lines.clear();
        dots.clear();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        for (Line line : lines){
            canvas.drawLine(line.getX1(),line.getY1(),line.getX2(),line.getY2(), linePaint);
        }
        for (Dot dot : dots){
            canvas.drawCircle(dot.getX(),dot.getY(),radius, dotPaint);
        }
    }

}
