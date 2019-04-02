package com.google.ar.core.examples.java.helloar;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;


class ImageProcessing {

    static private final int numOfDirection = 4;


    static ArrayList<double[]> detectDoor(Mat mRgba) {

//        Size size = new Size(640, 480);
//        Imgproc.resize(mRgba, mRgba, size);
        Mat lines = new Mat();
        ImageProcessing.houghLines(mRgba, lines);

        Point[] edges = new Point[numOfDirection]; // 0-topLeft 1-topRight 2-bottomLeft 3-bottomRight
        for (int i = 0; i < edges.length; i++)
            edges[i] = new Point(0, 0);
        ImageProcessing.detectCorners(mRgba, edges);

        ArrayList<double[]> vecs = new ArrayList<>();

        for (int x = 0; x < lines.rows(); x++) {
            double[] vec = lines.get(x, 0);
            vecs.add(vec);
//        Point[] pointsOfIntersection = getPointsOfIntersection(lines);
        }
        return vecs;
    }


    private static void detectCorners(Mat initMat, Point[] points) {

        int width = initMat.width();
        int height = initMat.height();

        int widthForSearch = width / 8;
        int heightForSearch = height / 8;

        Rect roiTopLeft = new Rect(0, 0, widthForSearch, heightForSearch);
        Rect roiTopRight = new Rect(width - widthForSearch, 0, widthForSearch, heightForSearch);
        Rect roiBottomLeft = new Rect(0, height - heightForSearch, widthForSearch, heightForSearch);
        Rect roiBottomRight = new Rect(width - widthForSearch, height - heightForSearch, widthForSearch, heightForSearch);

        harris(roiTopLeft, initMat, points[0]);
        harris(roiTopRight, initMat, points[1]);
        harris(roiBottomLeft, initMat, points[2]);
        harris(roiBottomRight, initMat, points[3]);

    }


    private static void houghLines(Mat initMat, Mat lines) {
        Mat initImg; // initial image
        Mat greyImg; // converted to grey

        int threshold = 50;

        // minimal length of line is defined to be 1/4 of min(width,height
        double minLineSize = Math.min(initMat.width(), initMat.height()) / 4.0;
        int lineGap = 10;

        initImg = initMat;
        greyImg = new Mat();
        Imgproc.cvtColor(initImg, greyImg, Imgproc.COLOR_BGR2GRAY);

        Imgproc.blur(greyImg, greyImg, new Size(3.d, 3.d));
        Imgproc.adaptiveThreshold(greyImg, greyImg, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY_INV, 15, 4);

        Imgproc.HoughLinesP(greyImg, lines, 1, Math.PI / 180, threshold,
                minLineSize, lineGap);

        linesFilter linesFilter = new linesFilter();
        lines = linesFilter.LinesFilter(greyImg, lines);

        for (int x = 0; x < lines.rows(); x++) {
            double[] vec = lines.get(x, 0);
            double x1 = vec[0],
                    y1 = vec[1],
                    x2 = vec[2],
                    y2 = vec[3];
            Point start = new Point(x1, y1);
            Point end = new Point(x2, y2);

            // draw the line on the init image
//            Imgproc.line(initImg, start, end, new Scalar(0, 255, 0, 255), 5);
        }
    }


    private static void harris(Rect partOfImage, Mat original, Point point) {

        // This function implements the Harris Corner detection. The corners at intensity > thresh
        // are drawn.
        Mat cropped = new Mat(original, partOfImage);
        Mat harris_scene = new Mat(), imgGray = new Mat();

        int x = partOfImage.x;
        int y = partOfImage.y;

        Imgproc.cvtColor(cropped,imgGray,Imgproc.COLOR_RGB2GRAY);

        int blockSize = 2;
        int apertureSize = 3;
        double k = 0.04;
        Imgproc.cornerHarris(imgGray, harris_scene, blockSize, apertureSize, k);

        System.out.println("width: " + harris_scene.width() + " height: " + harris_scene.height());

        double maxVal = Core.minMaxLoc(harris_scene).maxVal;
        double thresh = 1.0 * maxVal;

        for( int j = 0; j < harris_scene.rows() ; j++){
            for( int i = 0; i < harris_scene.cols(); i++){
                if (harris_scene.get(j, i)[0] >= thresh){
                    point.x = x + i;
                    point.y = y + j;
                    Imgproc.circle(original, new Point(point.x, point.y), 3 , new Scalar(0, 0, 255, 255), 2 ,8 , 0);
                }
            }
        }
    }


    private static Point lineLineIntersection(Point A, Point B, Point C, Point D) {
        // Line AB represented as a1x + b1y = c1
        double a1 = B.y - A.y;
        double b1 = A.x - B.x;
        double c1 = a1*(A.x) + b1*(A.y);

        // Line CD represented as a2x + b2y = c2
        double a2 = D.y - C.y;
        double b2 = C.x - D.x;
        double c2 = a2*(C.x)+ b2*(C.y);

        double determinant = a1*b2 - a2*b1;

        if (determinant == 0)
        {
            // The lines are parallel. This is simplified
            // by returning a pair of FLT_MAX
            return new Point(Double.MAX_VALUE, Double.MAX_VALUE);
        }
        else
        {
            double x = (b2*c1 - b1*c2)/determinant;
            double y = (a1*c2 - a2*c1)/determinant;
            return new Point(x, y);
        }
    }


    static Point[] getPointsOfIntersection(Mat lines) {
//        Point[] linesEdges = new Point[numOfDirection * 2];
        Point[] linesEdges = new Point[2];

        for (int x = 0; x < lines.rows(); x++) {
            double[] vec = lines.get(x, 0);
            double x1 = vec[0], y1 = vec[1], x2 = vec[2], y2 = vec[3];
            linesEdges[2 * x] = new Point(x1, y1);
            linesEdges[2 * x + 1] = new Point(x2, y2);
        }
        Point[] pointsOfIntersection = new Point[numOfDirection];

        for (int i = 0; i < numOfDirection; i++) {
            int p1Index = 2 * i;
            int p2Index = 2 * i + 1;
            int p3Index = (2 * i + 2) % (2 * numOfDirection);
            int p4Index = (2 * i + 3) % (2 * numOfDirection);

            pointsOfIntersection[i] = lineLineIntersection(linesEdges[p1Index], linesEdges[p2Index], linesEdges[p3Index], linesEdges[p4Index]);
        }
        return pointsOfIntersection;
    }
}
