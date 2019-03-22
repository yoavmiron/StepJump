package com.google.ar.core.examples.java.helloar;

import android.graphics.Bitmap;


import org.bytedeco.javacpp.opencv_core.Mat;



public class CvProc {
//    private Mat mat1;
    public CvProc() {
//        mat1= new Mat();
    }


    private Mat bitMapToMat(Bitmap bitmap){
        Bitmap bmp32 = bitmap.copy(Bitmap.Config.ARGB_8888, true);
        Mat mat = new Mat();
        bitmapToMat(bmp32, mat);
        return mat;

    }
    public int process(Bitmap bitmap){
        Mat mat = bitmapToMat(bitmap);
        int a=1;
        // do what you want with the mat
        return 1;
    }

    public void createMat(){
        Mat mat= new Mat();
        int a=1;
    }
}
