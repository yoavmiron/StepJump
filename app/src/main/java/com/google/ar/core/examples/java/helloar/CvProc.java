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

    private void bitmapToMat(Bitmap bitmap, Mat mat, int orientation){
        Bitmap bmp32 = bitmap.copy(Bitmap.Config.ARGB_8888, true);
        bmp32 = rotateImage(bmp32, orientation);
        Utils.bitmapToMat(bmp32, mat);

    }
    double[][] process(Bitmap bitmap, float top,float bottom, float left, float right, int orientation){
        Mat mat= new Mat();
        bitmapToMat(bitmap, mat, orientation);
        mat = mat.submat((int)left,(int)right,(int)top, (int)bottom);
        double[][] ret = ImageProcessing.getLinesOfDoor(mat);
        mat.release();
        return ret;
    }

    public static Bitmap rotateImage(Bitmap src, float degree)
    {
        // create new matrix
        Matrix matrix = new Matrix();
        // setup rotation degree
        matrix.postRotate(degree);
        return Bitmap.createBitmap(src, 0, 0, src.getWidth(), src.getHeight(), matrix, true);
    }
}
