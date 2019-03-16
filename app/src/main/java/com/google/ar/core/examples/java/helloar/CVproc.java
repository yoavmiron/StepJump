package com.google.ar.core.examples.java.helloar;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.media.Image;



//import org.opencv.android.Utils;
//import org.opencv.core.Mat;
//import org.opencv.imgproc.Imgproc;

import com.quickbirdstudios.yuv2mat.extensions.Yuv;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;

public class CVproc {
    private Bitmap rgbFrameBitmap = null;
    private Bitmap croppedBitmap = null;
    private Matrix frameTocropTransform;
    private int[] rgbBytes = null;
    public CVproc() {
    }
    public void proccess(Image image){
//        Bitmap bitmap = convert_image_to_bitmap(image);
//        Mat img = convert_bitmap_to_mat(bitmap)
        try {
            Mat img = Yuv.toMat(image);
        }catch (Throwable e){
            Throwable a=e;
        }

        Mat gray = new Mat();
//        Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);

    }

//
//    private Mat convert_bitmap_to_mat(Bitmap myBitmap){
//
//        Mat mat = new Mat();
//        Bitmap bmp32 = myBitmap.copy(Bitmap.Config.ARGB_8888, true);
//        Utils.bitmapToMat(bmp32, mat);
//
//        return mat;
//    }
//
//    private Bitmap convert_image_to_bitmap(Image image){
//
//        ByteBuffer buffer = image.getPlanes()[0].getBuffer();
//        byte[] bytes = buffer.array();
//        Bitmap myBitmap = BitmapFactory.decodeByteArray(bytes,0,bytes.length,null);
//
//        return myBitmap;
//    }
}



