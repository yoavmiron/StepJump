package com.google.ar.core.examples.java.helloar;

import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.graphics.RectF;
import android.media.Image;

import org.tensorflow.lite.Interpreter;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Vector;

public class OCD {
    // Only return this many results.
    private static final int NUM_DETECTIONS = 10;
    private final int height;
    private final int width;
    private boolean isModelQuantized;
    // Float model
    private static final float IMAGE_MEAN = 128.0f;
    private static final float IMAGE_STD = 128.0f;
    // Number of threads in the java app
    private static final int NUM_THREADS = 8;
    // Config values.
    private int inputSize;
    // Pre-allocated buffers.
    private Vector<String> labels = new Vector<String>();
    private int[] intValues;
    // outputLocations: array of shape [Batchsize, NUM_DETECTIONS,4]
    // contains the location of detected boxes
    private float[][][] outputLocations;
    // outputClasses: array of shape [Batchsize, NUM_DETECTIONS]
    // contains the classes of detected boxes
    private float[][] outputClasses;
    // outputScores: array of shape [Batchsize, NUM_DETECTIONS]
    // contains the scores of detected boxes
    private float[][] outputScores;
    // numDetections: array of shape [Batchsize]
    // contains the number of detected boxes
    private float[] numDetections;

    int cropSize = 300;

    private ByteBuffer imgData;

    private Interpreter tfLite;

    private Bitmap rgbFrameBitmap = null;
    private Bitmap croppedBitmap = null;
    private Bitmap cropCopyBitmap = null;
    private Matrix frameTocropTransform;
    private Matrix cropToFrameTransform;
    private Runnable imageConverter;
    private int[] rgbBytes = null;
    private CvProc cvProc;

    /**
     * Class that represents a recognition in an image
     */
    public class Recognition {
        final RectF location;
        final String label;
        final float confidence;

        public Recognition(RectF location, String label, float confidence) {
            this.location = location;
            this.label = label;
            this.confidence = confidence;
        }

        public String getLabel() {
            return label;
        }

        public Float getConfidence() {
            return confidence;
        }

        public RectF getLocation() {
            return new RectF(location);
        }

    }


    private OCD(int width, int height, int orientation) {
        this.width = width;
        this.height = height;
        rgbFrameBitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        croppedBitmap = Bitmap.createBitmap(cropSize, cropSize, Bitmap.Config.ARGB_8888);
        frameTocropTransform = ImageUtils.getTransformationMatrix(width, height, cropSize, cropSize,
                orientation + 90, false);
        cropToFrameTransform = new Matrix();
        frameTocropTransform.invert(cropToFrameTransform);
        rgbBytes = new int[width * height];
        cvProc = new CvProc();
    }

    public static OCD create(final AssetManager assetManager,
                             final String modelFilename,
                             final String labelFilename,
                             final int inputSize,
                             final boolean isQuantized,
                             final int width,
                             final int height,
                             final int orientation) throws IOException {
        final OCD ocd = new OCD(width, height, orientation);
        ocd.tfLite = new Interpreter(loadModelFile(assetManager, modelFilename));
        ocd.tfLite.setNumThreads(NUM_THREADS);
        BufferedReader br = null;
        InputStream labelsInput = null;
        ocd.inputSize = inputSize;
        ocd.isModelQuantized = isQuantized;
        labelsInput = assetManager.open(labelFilename);
        br = new BufferedReader(new InputStreamReader(labelsInput));
        String line;
        while ((line = br.readLine()) != null) {
            ocd.labels.add(line);
        }
        br.close();
        ocd.intValues = new int[inputSize * inputSize];
        int numBytesPerChannel;
        if (ocd.isModelQuantized) {
            numBytesPerChannel = 1; // Quantized
        } else {
            numBytesPerChannel = 4; // Floating point
        }
        ocd.imgData = ByteBuffer.allocateDirect(inputSize * inputSize * 3 * numBytesPerChannel);
        ocd.imgData.order(ByteOrder.nativeOrder());
        ocd.outputLocations = new float[1][NUM_DETECTIONS][4];
        ocd.outputClasses = new float[1][NUM_DETECTIONS];
        ocd.outputScores = new float[1][NUM_DETECTIONS];
        ocd.numDetections = new float[1];
        return ocd;
    }

    public ArrayList<Recognition> detect(final Image image) {
        prepareImage(image);
        croppedBitmap.getPixels(intValues, 0, croppedBitmap.getWidth(), 0, 0, croppedBitmap.getWidth(), croppedBitmap.getHeight());
        imgData.rewind();
        for (int i = 0; i < inputSize; ++i) {
            for (int j = 0; j < inputSize; ++j) {
                int pixelValue = intValues[i * inputSize + j];
                if (isModelQuantized) {
                    // Quantized model
                    imgData.put((byte) ((pixelValue >> 16) & 0xFF));
                    imgData.put((byte) ((pixelValue >> 8) & 0xFF));
                    imgData.put((byte) (pixelValue & 0xFF));
                } else { // Float model
                    imgData.putFloat((((pixelValue >> 16) & 0xFF) - IMAGE_MEAN) / IMAGE_STD);
                    imgData.putFloat((((pixelValue >> 8) & 0xFF) - IMAGE_MEAN) / IMAGE_STD);
                    imgData.putFloat(((pixelValue & 0xFF) - IMAGE_MEAN) / IMAGE_STD);
                }
            }
        }
        outputLocations = new float[1][NUM_DETECTIONS][4];
        outputClasses = new float[1][NUM_DETECTIONS];
        outputScores = new float[1][NUM_DETECTIONS];
        numDetections = new float[1];
        Object[] inputArray = {imgData};
        Map<Integer, Object> outputMap = new HashMap<>();
        outputMap.put(0, outputLocations);
        outputMap.put(1, outputClasses);
        outputMap.put(2, outputScores);
        outputMap.put(3, numDetections);

        // run model
        tfLite.runForMultipleInputsOutputs(inputArray, outputMap);

        // Show the best detections.
        // after scaling them back to the input size.
        final ArrayList<Recognition> recognitions = new ArrayList<>(NUM_DETECTIONS);
        for (int i = 0; i < NUM_DETECTIONS; i++) {
            if (outputScores[0][i] < 0.5) {
                continue;
            }
            final RectF detection =
                    new RectF(
                            outputLocations[0][i][1],
                            outputLocations[0][i][0],
                            outputLocations[0][i][3],
                            outputLocations[0][i][2]);
            // SSD Mobilenet V1 Model assumes class 0 is background class
            // in label file and class labels start from 1 to number_of_classes+1,
            // while outputClasses correspond to class index from 0 to number_of_classes
            int labelOffset = 1;
            recognitions.add(
                    new Recognition(
                            detection,
                            labels.get((int) outputClasses[0][i] + labelOffset),
                            outputScores[0][i]));
        }
        return recognitions;
    }


    /**
     * Memory-map the model file in Assets.
     */
    private static MappedByteBuffer loadModelFile(AssetManager assets, String modelFilename)
            throws IOException {
        AssetFileDescriptor fileDescriptor = assets.openFd(modelFilename);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    private void prepareImage(final Image image) {
        ByteBuffer cameraPlaneY = image.getPlanes()[0].getBuffer();
        ByteBuffer cameraPlaneU = image.getPlanes()[1].getBuffer();
        ByteBuffer cameraPlaneV = image.getPlanes()[2].getBuffer();

        byte[] compositeByteArray = new byte[cameraPlaneY.capacity() + cameraPlaneU.capacity() + cameraPlaneV.capacity()];
        cameraPlaneY.get(compositeByteArray, 0, cameraPlaneY.capacity());
        cameraPlaneV.get(compositeByteArray, cameraPlaneY.capacity(), cameraPlaneV.capacity());
        cameraPlaneU.get(compositeByteArray, cameraPlaneY.capacity() + cameraPlaneV.capacity(), cameraPlaneU.capacity());
//        cameraPlaneU.get(compositeByteArray, cameraPlaneY.capacity(), cameraPlaneU.capacity());
//        cameraPlaneV.get(compositeByteArray, cameraPlaneY.capacity() + cameraPlaneU.capacity(), cameraPlaneV.capacity());

        ImageUtils.convertYUV420SPToARGB8888(compositeByteArray, width, height, rgbBytes);
        rgbFrameBitmap.setPixels(rgbBytes, 0, width, 0, 0, width, height);
        final Canvas canvas = new Canvas(croppedBitmap);
        canvas.drawBitmap(rgbFrameBitmap, frameTocropTransform, null);
    }


    double[][] imageProcess(Image image, float top, float bottom, float left, float right, int orientation) {
        prepareImage(image);
        return cvProc.process(rgbFrameBitmap, top, bottom, left, right, orientation);
    }

    static float[] transformRatioToScreen(float top, float bottom, float left, float right, int realWidth, int screenWidth, int screenHeight) {
        top *= (float) screenHeight;
        bottom *= (float) screenHeight;
        int deltaW = realWidth - screenWidth;
        left *= (float) realWidth - deltaW / 2;
        right *= (float) realWidth - deltaW / 2;
        top = top > screenHeight - 1 ? screenHeight - 1 : top;
        top = top < 0 ? 0 : top;
        bottom = bottom > screenHeight - 1 ? screenHeight - 1 : bottom;
        bottom = bottom < 0 ? 0 : bottom;
        left = left > screenWidth - 1 ? screenWidth - 1 : left;
        left = left < 0 ? 0 : left;
        right = right > screenWidth - 1 ? screenWidth - 1 : right;
        right = right < 0 ? 0 : right;
        return new float[]{top, bottom, left, right};
    }
}
