package com.google.ar.core.examples.java.helloar;
import com.google.ar.core.Pose;

import java.lang.Math;
public class TwoDLine {
    private float a,b,c;
    // az+bx+c = 0
    // if a=0 b=1
    // otherwise a=1
    public TwoDLine(float a, float b, float c)
    {
        this.a=a;
        this.b=b;
        this.c=c;
    }

    public float getA() {
        return a;
    }

    public float getB() {
        return b;
    }

    public float getC() {
        return c;
    }

    static TwoDLine Create_Line_From_Incline_And_Point(float incline, float x0, float z0)
    {
        float a,b,c;
        a=1;
        b=-incline;
        c=-z0-b*x0;
        return new TwoDLine(a,b,c);
    }

    static TwoDLine Create_From_Two_Points(float x1,float z1, float x2, float z2)
    {
        if (x1==x2)
            return new TwoDLine(0,1,-x1);
        float a,b,c;
        a=1;
        b=-(z2-z1)/(x2-x1);
        c=-z2-b*x2;
        return new TwoDLine(a,b,c);
    }


    public static float Distance_Between_Points(float x1, float z1, float x2, float z2)
    {
        return (float)Math.sqrt((x1-x2)*(x1-x2)+(z1-z2)*(z1-z2));
    }


    public boolean is_intersection_in_between_points(float x1, float z1, float x2, float z2)
    {
        /*
        Checks if the intersection of the line defined by (x1,z1,x2,z2)
         */
        float[] intersect = this.Find_InterSection(Create_From_Two_Points(x1, z1, x2, z2));
        if (intersect[2] == 0)
            return false;
        if (intersect[2] == 2)
            return true; //should never happen, but might be
        if (intersect[2] == 1) {
            float x = intersect[0], z = intersect[1];
            float sum_distances = Math.abs(Distance_Between_Points(x1, z1, x, z)) + Math.abs(Distance_Between_Points(x2, z2, x, z));
            if (sum_distances > Math.abs(Distance_Between_Points(x1, z1, x2, z2)))
                return false;
            else
                return true;
        }
        return false;
    }

    public float[] Find_InterSection(TwoDLine other)
    {
        // returns {x,z,1 if 1 intersection found 0 if no intersection found and 2 if the 2 lines are
        // the same line}
        if (this.a==other.getA()&&this.b== other.getB())
        {
            //parallel or same line
            if (this.c==other.getC())
                return new float[]{0,0,2};
            else
                return new float[]{0,0,0};
        }
        else
        {
            if (this.a==0 || other.getA()==0)
            {
                //one is of the form x=-c
                if (this.a==0)
                    return new float[]{-this.c,-other.b*(-this.c)-other.c,1};
                else
                    return new float[]{-other.c,-this.b*(-other.c)-this.c,1};

            }
            else
            {
                //both are of the form z=-bx-c
                float x = (this.c-other.getC())/(other.getB()-this.b);
                float z = -this.b*x-this.c;
                return new float[]{x,z,1};
            }
        }
    }

    public TwoDLine Vertical(float x1, float z1)
    {
        /*
        Returns a line vertical to this line with an intersection with this line at (x1,z1)
         */
        if (this.a==0)
            return new TwoDLine(1,0,-z1);
        float incline = this.a/this.b;
        return Create_Line_From_Incline_And_Point(incline,x1,z1);
    }

    public float Distance_To_Point_Not_Abs(float x1,float z1)
    {
        /*
        Returns the distance of a point from the line (without absolute value - i.e.
        it is negative if the point is below the line and positive if the point is above the line)
         */
        float temp = (float)Math.sqrt((double)(this.a*this.a+this.b*this.b));
        return (this.a*z1+this.b*x1+c)/temp;
    }

    public static float Distance_Between_Intersections(TwoDLine l1, TwoDLine l2, TwoDLine l3)
    {
        /*
        finds the distance between the intersections of (l1 and l2) and (l1 and l3)
         */
        float[] p1 = l1.Find_InterSection(l2);
        float[] p2 = l1.Find_InterSection(l3);
        if (p1[2]!=1 || p2[2]!=1)
            return -1; //error - there is no exactly 1 intersection
        float dx = p2[0]-p1[0];
        float dz = p2[1]-p1[1];
        return (float)Math.sqrt(dx*dx+dz*dz);
    }

    public TwoDLine[] find_lines (float[] points)
    {
        /*
        Finds the two lines connecting the two pairs of points closest to this line from both sides
         */
        float[] xes = new float[points.length/2];
        float[] zes = new float[points.length/2];
        for(int i = 0; i < points.length; i+= 2)
        {
            xes[i/2] = points[i];
            zes[i/2] = points[i+1];
        }
        float distance_from_line = 0, pre_distance_from_line = 0;
        float[][] edge_points = new float[4][2];
        int count = 0;
        distance_from_line = this.b*xes[0] +this.a*zes[0] + this.c;
        for(int i = 0; i< xes.length; i++)
        {
            pre_distance_from_line = distance_from_line;
            distance_from_line = this.b*xes[i] +this.a*zes[i] + this.c;
            if((double)pre_distance_from_line*distance_from_line <= 0)
            {
                edge_points[count][0] = xes[i-1];
                edge_points[count][1] = zes[i-1];
                edge_points[count+1][0] = xes[i];
                edge_points[count+1][1] = zes[i];
                count += 2;
            }
            if (count>3)
                break;
        }
        TwoDLine line1 = Create_From_Two_Points(edge_points[0][0], edge_points[0][1],
                edge_points[1][0], edge_points[1][1]);
        TwoDLine line2 = Create_From_Two_Points(edge_points[2][0], edge_points[2][1],
                edge_points[3][0], edge_points[3][1]);
        TwoDLine[] lines2 = new TwoDLine[2];
        lines2[0] = line1;
        lines2[1] = line2;
        return lines2;
    }

    public static float Width_Of_Plane(float x1, float z1, float x2, float z2, float[] polygon_vertices)
    {
        /*
        Finds the width of the plane at point (x2,z2) where (x1,z1) is the center
         */
        TwoDLine l = Create_From_Two_Points(x1,z1,x2,z2);
        TwoDLine vertical = l.Vertical(x2,z2);
        TwoDLine[] lines = vertical.find_lines(polygon_vertices);
        float width = TwoDLine.Distance_Between_Intersections(vertical,lines[0],lines[1]);
        return width;
    }

    public static float find_width (float[] points)
    {
        float[] xes = new float[points.length/2];
        float[] zes = new float[points.length/2];
        for(int i = 0; i < points.length; i+= 2)
        {
            xes[i/2] = points[i];
            zes[i/2] = points[i+1];
        }
        float maxX = 0;
        for (int i=0; i< xes.length; i++)
        {
            if (xes[i] > maxX)
                maxX = xes[i];
        }
        float width;
        float min_X_width = 10000;
        for (float i=0; i<maxX - 1; i+=0.1)
        {
            width = Width_Of_Plane(0, 0, i, 0, points);
            if (width < min_X_width)
                min_X_width = width;
        }
        float maxZ = 0;
        for (int i=0; i< zes.length; i++)
        {
            if (zes[i] > maxZ)
                maxZ = zes[i];
        }
        width = 0;
        float min_Z_width = 10000;
        for (float i=0; i<maxZ - 1; i+=0.1)
        {
            width = Width_Of_Plane(0, 0, i, 0, points);
            if (width < min_Z_width)
                min_Z_width = width;
        }
        return min_X_width < min_Z_width ? min_X_width : min_Z_width;
    }

    public static float[] filter_plane_points(PointCloud cloud, float plane_y)
    {

        float[] points_on_plane = new float[cloud.getPoints().array().length];
        int index=0;
        for (int i =0; i<cloud.getPoints().array().length;i+=4) {
            float dy = cloud.getPoints().array()[i + 1] - plane_y;
            if (dy <= 0.2 && dy >= -0.2) {
                points_on_plane[index] = cloud.getPoints().array()[i];//x
                points_on_plane[index+1] = cloud.getPoints().array()[i+2];//z
                index+=2;
            }
        }
        float[] filtered = new float[index];
        for (int i =0; i<index;i++)
        {
            filtered[i]=points_on_plane[i];
        }
        return filtered;
    }

    public static float[] find_point_with_max_distance(float x_center_pose,float z_center_pose,float[] points)
    {
        //points - array of [x1,z1,x2,z2...]
        float max_distance = 0;
        float disance;
        float[] max_dist_point = new float[2];
        for (int i =2; i<points.length; i+=2)
        {
            disance=TwoDLine.Distance_Between_Points(x_center_pose,z_center_pose,points[i],points[i+1]);
            if (disance>max_distance)
            {
                max_dist_point[0]=points[i];//x
                max_dist_point[1]=points[i+1];//z
                max_distance = disance;
            }
        }
        return max_dist_point;
    }

    public static float Get_Rotation_Angle(float x_center_pose,float z_center_pose, float x_p, float z_p,Plane plane)
    {
        float x_r = x_p-x_center_pose;
        float z_r = z_p-z_center_pose;
        float[] farrest_from_polygon = TwoDLine.find_point_with_max_distance(0,0,plane.getPolygon().array());
        float xp = farrest_from_polygon[0];
        float zp = farrest_from_polygon[1];
        return (float)Math.asin((z_p-z_r*x_p/x_r)/(x_r+z_r*z_r/x_r));
    }

}

