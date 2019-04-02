package com.google.ar.core.examples.java.helloar;

import android.graphics.Bitmap;
import org.opencv.android.Utils;

import org.opencv.core.Mat;
import com.google.ar.core.examples.java.helloar.ImageProcessing;

import java.util.ArrayList;

public class CvProc {
    private Mat mat;
    public CvProc() {
        mat = new Mat();
    }

    private void bitmapToMat(Bitmap bitmap){
        Bitmap bmp32 = bitmap.copy(Bitmap.Config.ARGB_8888, true);
        Utils.bitmapToMat(bmp32, mat);

    }
    ArrayList<double[]> process(Bitmap bitmap){
        bitmapToMat(bitmap);
        return ImageProcessing.detectDoor(mat);
    }
}
