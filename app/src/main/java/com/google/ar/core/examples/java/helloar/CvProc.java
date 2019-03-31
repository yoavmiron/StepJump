package com.google.ar.core.examples.java.helloar;

import android.graphics.Bitmap;


//import org.bytedeco.javacpp.opencv_core.Mat;


import org.opencv.core.Mat;

import static org.opencv.android.Utils.bitmapToMat;


public class CvProc {
    private Mat mat;
    public CvProc() {
        mat= new Mat();
    }


    private Mat bitMapToMat(Bitmap bitmap){
        Bitmap bmp32 = bitmap.copy(Bitmap.Config.ARGB_8888, true);
        Mat mat = new Mat();
        bitmapToMat(bmp32, mat);
        return mat;

    }
    public int process(Bitmap bitmap){
//        Mat mat = new Mat();
        bitmapToMat(bitmap, mat);
        // do what you want with the mat
        mat.release();
        return 1;
    }

    public void createMat(){
        Mat mat= new Mat();
        int a=1;
    }
}
