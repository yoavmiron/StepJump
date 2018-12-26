package com.google.ar.core.examples.java.helloar;

import android.content.res.AssetFileDescriptor;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.YuvImage;
import android.media.Image;
import org.tensorflow.lite.Interpreter;
import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
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

public class OCD
{
    // Only return this many results.
    private static final int NUM_DETECTIONS = 10;
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

    private ByteBuffer imgData;

    private Interpreter tfLite;

    /**
     * Class that represents a recognition in an image
     */
    public class Recognition
    {
        final RectF location;
        final String label;
        final float confidence;

        public Recognition(RectF location, String label, float confidence)
        {
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


    private OCD() {}

    public static OCD create(final AssetManager assetManager,
        final String modelFilename,
        final String labelFilename,
        final int inputSize,
        final boolean isQuantized) throws IOException
    {
        final OCD ocd = new OCD();
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
        ocd.imgData = ByteBuffer.allocateDirect(1 * inputSize * inputSize * 3 * numBytesPerChannel);
        ocd.imgData.order(ByteOrder.nativeOrder());
        ocd.outputLocations = new float[1][NUM_DETECTIONS][4];
        ocd.outputClasses = new float[1][NUM_DETECTIONS];
        ocd.outputScores = new float[1][NUM_DETECTIONS];
        ocd.numDetections = new float[1];
        return ocd;
    }

    public ArrayList<Recognition> detect(final Image image)
    {
        Bitmap bitmap = prepareImage(image);
        bitmap.getPixels(intValues, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
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
        for(int i = 0; i < NUM_DETECTIONS; i++)
        {
            if(outputScores[0][i] < 0.5)
            {
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



    /** Memory-map the model file in Assets. */
    private static MappedByteBuffer loadModelFile(AssetManager assets, String modelFilename)
            throws IOException {
        AssetFileDescriptor fileDescriptor = assets.openFd(modelFilename);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    private Bitmap prepareImage(final Image image)
    {
        ByteBuffer cameraPlaneY = image.getPlanes()[0].getBuffer();
        ByteBuffer cameraPlaneU = image.getPlanes()[1].getBuffer();
        ByteBuffer cameraPlaneV = image.getPlanes()[2].getBuffer();

        byte[] compositeByteArray = new byte[cameraPlaneY.capacity() + cameraPlaneU.capacity() + cameraPlaneV.capacity()];
        cameraPlaneY.get(compositeByteArray, 0, cameraPlaneY.capacity());
        cameraPlaneU.get(compositeByteArray, cameraPlaneY.capacity(), cameraPlaneU.capacity());
        cameraPlaneV.get(compositeByteArray, cameraPlaneY.capacity() + cameraPlaneU.capacity(), cameraPlaneV.capacity());

        ByteArrayOutputStream baOutputStream = new ByteArrayOutputStream();
        YuvImage yuvImage = new YuvImage(compositeByteArray, ImageFormat.NV21, image.getWidth(), image.getHeight(), null);
        yuvImage.compressToJpeg(new Rect(0, 0, image.getWidth(), image.getHeight()), 75, baOutputStream);
        byte[] byteForBitmap = baOutputStream.toByteArray();
        Bitmap b =  BitmapFactory.decodeByteArray(byteForBitmap, 0, byteForBitmap.length);
        return Bitmap.createScaledBitmap(b, inputSize, inputSize, false);
    }
}
