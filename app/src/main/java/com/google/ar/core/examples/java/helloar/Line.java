package com.google.ar.core.examples.java.helloar;

public class Line {
    private float x1;
    private float y1;
    private float x2;
    private float y2;

    public Line(float cx1, float cy1, float cx2, float cy2) {
        x1 = cx1;
        y1 = cy1;
        x2 = cx2;
        y2 = cy2;
    }

    public float getX1() {
        return x1;
    }

    public float getY1() {
        return y1;
    }

    public float getX2() {
        return x2;
    }

    public float getY2() {
        return y2;
    }
}
