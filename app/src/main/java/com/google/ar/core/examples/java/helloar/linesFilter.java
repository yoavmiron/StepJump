package com.google.ar.core.examples.java.helloar;
import org.opencv.core.Mat;
import java.util.ArrayList;

/**
 * Created by t8258266 on 25/03/2019.
 */

enum Area {
    UP, DOWN, LEFT, RIGHT, NO_AREA
}

public class linesFilter {

    public final void lines_filter() {
        Mat proc_img = new Mat();
    }


    Mat LinesFilter(Mat img, Mat lines)  {

        ArrayList<Mat> lines_list = new ArrayList<>();
        ArrayList<Mat> RightLinesList = new ArrayList<>();
        ArrayList<Mat> LeftLinesList = new ArrayList<>();
        ArrayList<Mat> UpLinesList = new ArrayList<>();
        ArrayList<Mat> DownLinesList = new ArrayList<>();

        for (int x = 0; x < lines.rows(); x++) {
            double[] vec = lines.get(x, 0);
            double x1 = vec[0],
                    y1 = vec[1],
                    x2 = vec[2],
                    y2 = vec[3];
            Area LineArea = FindArea(x1, y1, x2, y2, img);
            switch (LineArea) {
                case LEFT:
                    LeftLinesList.add(lines.row(x));
                    break;
                case RIGHT:
                    RightLinesList.add(lines.row(x));
                    break;
                case UP:
                    UpLinesList.add(lines.row(x));
                    break;
                case DOWN:
                    DownLinesList.add(lines.row(x));
                    break;
            }
        }
//        lines.release();

//        Mat left_lines = ListToMat(LeftLinesList);
//        Mat leftLine=FindLeftLine(left_lines);
//        if (leftLine == null){
//            System.out.println("oops, empty list");
//        }
//        else{
//            lines_list.add(leftLine);
//        }

        if (LeftLinesList.size()!=0 )
            lines_list.add(LeftLinesList.get(0));
        else
            lines_list.add(null);
        if (RightLinesList.size()!=0 )
            lines_list.add(RightLinesList.get(0));
        else
            lines_list.add(null);
        if (UpLinesList.size()!=0 )
            lines_list.add(UpLinesList.get(0));
        else
            lines_list.add(null);
        if (DownLinesList.size()!=0 )
            lines_list.add(DownLinesList.get(0));
        else
            lines_list.add(null);

//        lines_list.addAll(LeftLinesList);
//        lines_list.addAll(RightLinesList);
//        lines_list.addAll(UpLinesList);
//        lines_list.addAll(DownLinesList);

        if(!lines.empty()) {
            Mat filtered_lines = new Mat(0, 4, lines.type());

            for (int x = 0; x < lines_list.size(); x++) {
                if(lines_list.get(x) != null)
                    filtered_lines.push_back(lines_list.get(x));
            }
            return filtered_lines;
        }
        return lines;
    }


    private Area FindArea(double x1, double y1, double x2, double y2, Mat img) {
        double rows = img.size().height;
        double cols = img.size().width;
        double left = cols / 6;
        double right = cols - cols / 6;
        double up = rows / 10;
        double down = rows - rows / 10;

        if (y1 > down && y2 > down)
            return Area.DOWN;

        if (y1 < up && y2 < up)
            return Area.UP;

        if (x1 < left && x2 < left)
            return Area.LEFT;

        if (x1 > right && x2 > right)
            return Area.RIGHT;


        return Area.NO_AREA;
    }


    private double FindMeanPlace(double x1, double y1, double x2, double y2, Area area) {
        switch (area) {
            case DOWN:
                return (y1 + y2) / 2;
            case UP:
                return (y1 + y2) / 2;
            case RIGHT:
                return (x1 + x2) / 2;
            case LEFT:
                return (x1 + x2) / 2;
        }
        return 0;
    }


    private Mat FindLeftLine(Mat left_lines_list) {
        if  (left_lines_list.empty())
            return null;
        ArrayList<Mat> LeftLine = new ArrayList<>();

        double mean = 0;

        for (int x = 0; x < left_lines_list.rows(); x++) {
            double[] vec = left_lines_list.get(x,0);
            double x1 = vec[0],
                    y1 = vec[1],
                    x2 = vec[2],
                    y2 = vec[3];
            double new_mean = FindMeanPlace(x1, y1, x2, y2, Area.LEFT);
            if (new_mean > mean) {
                LeftLine.add(left_lines_list.row(x));
                mean = new_mean;
                //System.out.println(mean);
            }
        }

//        System.out.println(mean);
        return LeftLine.get(0);
    }


    private Mat ListToMat (ArrayList<Mat> LinesList) {
        if (LinesList.size() > 0) {
            Mat mat = new Mat(0, 4, LinesList.get(0).type());
            for (int x = 0; x < LinesList.size(); x++) {
                mat.push_back(LinesList.get(x));
            }
            return mat;
        }
        return new Mat();
    }
}

