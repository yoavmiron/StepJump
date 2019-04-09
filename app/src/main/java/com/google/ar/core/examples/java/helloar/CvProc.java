package com.google.ar.core.examples.java.helloar;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Matrix;
import android.widget.ImageView;

import org.opencv.android.Utils;

import org.opencv.core.Mat;

import java.util.ArrayList;

public class CvProc {

    public CvProc() {

    }

    private void bitmapToMat(Bitmap bitmap, Mat mat, float top, float bottom, float left, float right, int orientation) {
        Bitmap bmp32 = bitmap.copy(Bitmap.Config.ARGB_8888, true);
        bmp32 = rotateResizeImage(bmp32, top, bottom, left, right, orientation);
        Utils.bitmapToMat(bmp32, mat);

    }

    double[][] process(Bitmap bitmap, float top, float bottom, float left, float right, int orientation) {
        Mat mat = new Mat();
        orientation += 90;
        bitmapToMat(bitmap, mat, top, bottom, left, right, orientation);
        double[][] ret = ImageProcessing.getLinesOfDoor(mat);
        mat.release();
        return ret;
    }

    public static Bitmap rotateResizeImage(Bitmap src, float top, float bottom, float left, float right, float degree) {
        // create new matrix
        Matrix matrix = new Matrix();
        // setup rotation degree
        matrix.postRotate(degree);
        Bitmap bmp = Bitmap.createBitmap(src, 0, 0, src.getWidth(), src.getHeight(), matrix, true);
        return Bitmap.createBitmap(bmp, (int) left,(int) top, (int) (right - left), (int) (bottom - top), null, true);
    }
}
