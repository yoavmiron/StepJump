package com.google.ar.core.examples.java.helloar;

import android.view.MotionEvent;

import com.google.ar.core.Anchor;
import com.google.ar.core.Camera;
import com.google.ar.core.Frame;
import com.google.ar.core.HitResult;
import com.google.ar.core.Plane;
import com.google.ar.core.PointCloud;
import com.google.ar.core.Pose;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ConcurrentMap;

import static java.lang.Float.max;

public class AR {
    /**
     * computes the distance between to poses
     *
     * @param pose1 first pose
     * @param pose2 second pose
     * @return the distance between the poses
     */
    static float distanceBetweenPoses(Pose pose1, Pose pose2) {
        float dx = pose1.tx() - pose2.tx();
        float dy = pose1.ty() - pose2.ty();
        float dz = pose1.tz() - pose2.tz();

        // Compute the straight-line distance.
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    static float horizontalDistanceBetweenPoses(Pose pose1, Pose pose2) {
        float dx = pose1.tx() - pose2.tx();
        float dz = pose1.tz() - pose2.tz();

        // Compute the straight-line distance.
        return (float) Math.sqrt(dx * dx + dz * dz);
    }

    /**
     * computes the distance between to poses
     *
     * @param pose1 first pose
     * @param pose2 second pose
     * @return the distance between the poses
     */
    static float distanceBetweenPoses(float[] pose1, float[] pose2) {
        float dx = pose1[0] - pose2[0];
        float dy = pose1[1] - pose2[1];
        float dz = pose1[2] - pose2[2];

        // Compute the straight-line distance.
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * @param points all points recognized in the frame
     * @param floor  the floor plane
     * @return ArrayList of the objects in the session
     */
    static ArrayList<ArrayList<float[]>> getObjects(float[] points, Plane floor) {
        if (floor == null)
            return new ArrayList<>();
        ArrayList<float[]> allPoints = new ArrayList<>();
        for (int i = 0; i < points.length; i += 4) {
            float[] currPoint = {points[i], points[i + 1], points[i + 2], points[i + 3]};
            //Pose pointPose = new Pose(currPoint, new float[]{0.0f, 0.0f, 0.0f, 0.0f});
            // makes sure the point isn't on the floor
            if (currPoint[1] < floor.getCenterPose().getTranslation()[1] + 0.05) {
                continue;
            }
            if (currPoint[3] > 0.3) {
                allPoints.add(currPoint);
            }
        }
        ArrayList<ArrayList<float[]>> objects = new ArrayList<>();
        ArrayList<Integer> checked_indexes = new ArrayList<>();
        float distanceThreshold = 0.5f;
        int object_index = -1;
        int point_index = -1;
        int next_point = 0;
        while (checked_indexes.size() != allPoints.size()) {
            if (object_index == -1 || objects.get(object_index).size() <= point_index) {
                ArrayList<float[]> object = new ArrayList<>();
                while (checked_indexes.contains(next_point)) {
                    next_point++;
                }
                object.add(allPoints.get(next_point));
                objects.add(object);
                object_index++;
                next_point++;
                point_index = 0;
                continue;
            }
            ArrayList<float[]> object = objects.get(object_index);
            while (object.size() > point_index) {
                for (int i = 0; i < allPoints.size(); i++) {
                    if(checked_indexes.contains(i))
                        continue;
                    if(distanceBetweenPoses(object.get(point_index),allPoints.get(i)) < distanceThreshold){
                        object.add(allPoints.get(i));
                        checked_indexes.add(i);
                    }
                }
                point_index++;
            }
        }
        return objects;
    }


    /**
     * @param point     point to add to allPoints
     * @param allPoints all the points added so far
     * @return the index to which we need to add the new point based on X axis
     */
    private static int binarySearch(float[] point, ArrayList<float[]> allPoints) {
        if (allPoints.size() == 0) {
            return 0;
        }
        int min = 0;
        int max = allPoints.size();
        float pointX = point[0];
        while (min != max) {
            float midX = allPoints.get((min + max) / 2)[0];
            if (midX == pointX) {
                return (min + max) / 2;
            } else if (midX > pointX) {
                max = (max + min) / 2;
            } else {
                if (max - min == 1) {
                    return max;
                }
                min = (max + min) / 2;
            }
        }
        return min;
    }

    /**
     * @param point   the point to search it's object
     * @param objects all objects found
     * @return index of object in objects in which the anchor is placed
     */
    public static int getIndexOfObject(float[] point, ArrayList<ArrayList<float[]>> objects) {
        for (ArrayList<float[]> object : objects) {
            boolean contains = false;
            for (float[] pose1 : object) {
                if (pose1[0] == point[0] && pose1[1] == point[1] && pose1[2] == point[2]) {
                    contains = true;
                    break;
                }
            }
            if (contains) {
                return objects.indexOf(object);
            }
        }
        return -1;
    }

    /**
     * calculates the distance between two points on the screen in the real world
     *
     * @param pixel1 first pixel on screen
     * @param pixel2 second pixel on screen
     * @param frame  current frame
     * @return distance between two pixels in the real world, -1 if can't calculate
     */
    public static float pixelsToDistance(float[] pixel1, float[] pixel2, Frame frame) {
        List<HitResult> hits1 = frame.hitTest(pixel1[0], pixel1[1]);
        List<HitResult> hits2 = frame.hitTest(pixel2[0], pixel2[1]);
        if (hits1.isEmpty() || hits2.isEmpty()) {
            return -1.0f;
        }
        // set the minimal distance to be too large
        float minDistance = 300.0f;
        float sumDistance = 0.0f;
        ArrayList<Float> distances = new ArrayList<>();
        for (HitResult hit1 : hits1) {
            for (HitResult hit2 : hits2) {
                float distance = distanceBetweenPoses(hit1.getHitPose(), hit2.getHitPose());
                distances.add(distance);
                minDistance = (distance < minDistance) ? distance : minDistance;
                sumDistance += distance;
            }
        }
        Comparator<Float> c = (a, b) -> {
            if (a > b) {
                return 1;
            }
            return a.equals(b) ? 0 : -1;
        };
        //distances.sort(c);
        //return distances.get(0);
        return sumDistance / hits1.size() / hits2.size();
    }

    /**
     * calculates the distance between the given point and the camera
     *
     * @param pixel pixel on screen
     * @param frame current frame
     * @return distance between two pixels in the real world, -1 if can't calculate
     */
    static float pixelToDistance(float[] pixel, Frame frame) {
        List<HitResult> hits = frame.hitTest(pixel[0], pixel[1]);
        if (hits.isEmpty()) {
            return -1.0f;
        }
        // set the minimal distance to be too large
        float minDistance = 300.0f;
        Pose cameraPose = frame.getCamera().getPose();
        for (HitResult hit1 : hits) {
            float distance = distanceBetweenPoses(hit1.getHitPose(), cameraPose);
            minDistance = (distance < minDistance) ? distance : minDistance;
        }
        return minDistance;
    }

    /**
     * @param planes a list of all planes detected by the program
     * @return the floor if detected, null otherwise
     */
    static Plane getFloor(ArrayList<Plane> planes, Camera camera) {
        if (planes.size() == 0) {
            return null;
        }
        Plane floor = null;
        ArrayList<Plane> floors = new ArrayList<>();
        for (Plane plane : planes) {
            // if the plane is facing up
            if (plane.getType() == Plane.Type.HORIZONTAL_UPWARD_FACING) {
                Pose center = plane.getCenterPose();
                // if the plane is at least a meter below the person using the app
                float height = camera.getPose().getTranslation()[1] - center.getTranslation()[1];
                if (height > 0.6 && height < 2) {
                    if (floor == null) {
                        floor = plane;
                    } else if (max(floor.getExtentZ(), floor.getExtentX()) < max(plane.getExtentX(), plane.getExtentZ())) {
                        // if this plane if bigger in one of his dimensions than the largest of the current found
                        // floor's dimensions than it's more likely to be the actual floor
                        floor = plane;
                    }
                }
            }
        }
        return floor;
    }

    private static float Width_Of_Plane(float x1, float z1, float x2, float z2, float[] polygon_vertices) {
        /*
        Finds the width of the plane at point (x2,z2) where (x1,z1) is the center
         */
        TwoDLine l = TwoDLine.Create_From_Two_Points(x1, z1, x2, z2);
        TwoDLine vertical = l.Vertical(x2, z2);
        TwoDLine[] lines = vertical.find_lines(polygon_vertices);
        return TwoDLine.Distance_Between_Intersections(vertical, lines[0], lines[1]);
    }

    /**
     * calculates the width of a given plane
     *
     * @param floor plane to calculate it's width
     * @return width of plane if can calculate, 1000 otherwise
     */
    static float find_width(float[] points) {
        float[] xes = new float[points.length / 2];
        float[] zes = new float[points.length / 2];
        for (int i = 0; i < points.length; i += 2) {
            xes[i / 2] = points[i];
            zes[i / 2] = points[i + 1];
        }
        float maxX = 0;
        for (int i = 0; i < xes.length; i++) {
            if (xes[i] > maxX)
                maxX = xes[i];
        }
        float width;
        float min_X_width = 10000;
        for (float i = -maxX + 0.5f; i <= maxX - 0.5f; i += 0.1) {
            width = Width_Of_Plane(0, 0, i, 0, points);
            if (width == -1)
                continue;
            if (width < min_X_width)
                min_X_width = width;
        }
        float maxZ = 0;
        for (int i = 0; i < zes.length; i++) {
            if (zes[i] > maxZ)
                maxZ = zes[i];
        }
        float min_Z_width = 10000;
        for (float i = -maxZ + 0.5f; i < maxZ - 0.5f; i += 0.1) {
            width = Width_Of_Plane(0, 0, 0, i, points);
            if (width == -1)
                continue;
            if (width < min_Z_width)
                min_Z_width = width;
        }
        return min_X_width < min_Z_width ? min_X_width : min_Z_width;
    }

    private static ArrayList<ArrayList<HitResult>> findAllLeftRightOnLines(float[] pixelLeftUp, float[] pixelRightUp, float[] pixelLeftDown, float[] pixelRightDown, Frame frame) {
        float right_deltaY = pixelRightDown[1] - pixelRightUp[1];
        float right_deltaX = pixelRightDown[0] - pixelRightUp[0];
        float left_deltaY = pixelLeftDown[1] - pixelLeftUp[1];
        float left_deltaX = pixelLeftDown[0] - pixelLeftUp[0];
        ArrayList<HitResult> allRight = new ArrayList<>();
        ArrayList<HitResult> allLeft = new ArrayList<>();
        float num_hits = 50f;
        // get all locations on left side and right side
        for (float i = 0; i < num_hits; i++) {
            allRight.addAll(frame.hitTest(pixelRightUp[0] + right_deltaX * i / num_hits, pixelRightUp[1] + right_deltaY * i / num_hits));
            allLeft.addAll(frame.hitTest(pixelLeftUp[0] + left_deltaX * i / num_hits, pixelLeftUp[1] + left_deltaY * i / num_hits));
        }
        ArrayList<ArrayList<HitResult>> res = new ArrayList<>();
        res.add(allRight);
        res.add(allLeft);
        return res;
    }

    public static float sameHeightBetweenLines(float[] pixelLeftUp, float[] pixelRightDown, Frame frame) {
        return -1.0f;
    }

    static float findMinDistBetweenLines(float[] pixelLeftUp, float[] pixelRightUp, float[] pixelLeftDown, float[] pixelRightDown, Frame frame) {
        ArrayList<ArrayList<HitResult>> rightLeft = findAllLeftRightOnLines(pixelLeftUp, pixelRightUp, pixelLeftDown, pixelRightDown, frame);
        ArrayList<HitResult> allRight = rightLeft.get(0);
        ArrayList<HitResult> allLeft = rightLeft.get(1);
        float minDist = 300f;
        Pose cameraPose = frame.getCamera().getPose();
        for (HitResult r : allRight) {
            Pose rPose = r.getHitPose();
            if (distanceBetweenPoses(rPose, cameraPose) > 5) // caught a point behind the door
                continue;
            for (HitResult l : allLeft) {
                Pose lPose = l.getHitPose();
                if (distanceBetweenPoses(lPose, cameraPose) > 5) // caught a point behind the door
                    continue;
                float dis = horizontalDistanceBetweenPoses(rPose, lPose);
                minDist = dis < minDist ? dis : minDist;
            }
        }
        if (minDist == 300f) {
            return -1.0f;
        }
        return minDist;
    }

    static float[] bufferToArray(FloatBuffer f) {
        float[] points = new float[f.remaining()];
        for (int i = 0; i < points.length; i++) {
            points[i] = f.get();
        }
        f.rewind();
        return points;
    }

    /**
     * @param floor
     * @param points all point found in frame
     * @return list of all possible points for floor extension
     */
    static ArrayList<float[]> findExtensionPoints(Plane floor, float[] points) {
        final float certaintyThreshold = 0.10f; // was 0.15
        final float heightVariance = 0.05f;

        ArrayList<float[]> retList = new ArrayList<>();

        float floorHeight = floor.getCenterPose().ty();

        for (int i = 0; i < points.length; i += 4) {
            if (points[i + 3] < certaintyThreshold) {
                continue;
            }
            if (Math.abs(points[i + 1] - floorHeight) > heightVariance) {
                continue;
            }
            float[] point = new float[]{points[i], points[i + 1], points[i + 2]};
            Pose p = new Pose(point, new float[4]);
            if (!floor.isPoseInPolygon(p)){
                retList.add(point);
            }
        }
        return retList;
    }

    static float[][] polygon_to_xz(float[] points) {
        float[] xes = new float[points.length / 2];
        float[] zes = new float[points.length / 2];
        for (int i = 0; i < points.length; i += 2) {
            xes[i / 2] = points[i];
            zes[i / 2] = points[i + 1];
        }
        float[][] xz = new float[2][];
        xz[0] = xes;
        xz[1] = zes;
        return xz;
    }

    /**
     * @param floorPolygon    the polygon describing the floor in 2D
     * @param extensionPoints points to add to the polygon as vertices
     * @return the new polygon with (hopefully) optimal addition of points
     */
    static float[] extendFloor(float[] floorPolygon, ArrayList<float[]> extensionPoints, float x_extent, float z_extent) {

        /*
        float[][] xz = polygon_to_xz(floorPolygon);
        float[] x = xz[0];
        float[] z = xz[1];
        */

        float[][] ret = polygon_to_xz(floorPolygon);

        for (float[] point : extensionPoints) {
            ret = extendSinglePoint(ret, point[0], point[1], doubleArrayToSingle(ret),x_extent,z_extent);
        }

        return doubleArrayToSingle(ret);
    }

    /**
     * @param polygon array of the polygon's vertices (polygon[0] contains x values, polygon[1]
     *                contains z values)
     * @param x0      x coordinate of added point
     * @param z0      z coordinate of added point
     * @param xzForm  the same polygon in format [x1, z1, x2, z2,...]
     * @return the input polygon with the new point added, as a 2D array
     */
    private static float[][] extendSinglePoint(float[][] polygon, float x0, float z0,
                                               float[] xzForm, float x_extent, float z_extent) {

        float[] x = polygon[0];
        float[] z = polygon[1];
        int oldLen = x.length;
        if(Math.abs(x0) > x_extent/2 + 1f || Math.abs(z0) > z_extent/2 + 1f){
            return polygon;
        }

        for (int i = 0; i < oldLen; i++) { // check if new point is already a vertex
            if (x[i] == x0 && z[i] == z0) {
                return polygon;
            }
        }


        ArrayList<Integer> replaceFroms = new ArrayList<>();
        ArrayList<Integer> replaceTos = new ArrayList<>();


        /* loop over existing vertices: check whether the connecting line between the vertex and
         * (x0, y0) intersects the polygon and set replaceFrom, replaceTo accordingly */
        boolean didPreviousIntersect;
        boolean doesCurrentIntersect = doesConnectingLineIntersectPolygon(x0, z0, x[oldLen - 1],
                z[oldLen - 1], xzForm);
        for (int i = 0; i < oldLen; i++) {
            didPreviousIntersect = doesCurrentIntersect;
            doesCurrentIntersect = doesConnectingLineIntersectPolygon(x0, z0, x[i], z[i], xzForm);

            if (doesCurrentIntersect && (!didPreviousIntersect)) {
                int temp = i - 1;
                temp += (temp >= 0) ? 0 : oldLen;
                replaceTos.add(temp);
            } else if ((!doesCurrentIntersect) && didPreviousIntersect) {
                int temp = i + 1;
                temp -= (temp < oldLen) ? 0 : oldLen;
                replaceFroms.add(temp);
            }
        }

        int max = 0;
        int replaceFrom = 0; // indices  of vertices to be replaced by the new point (in Pythonian:
        int replaceTo = 0; // polygon[replaceFrom : replaceTo] = [(x0, z0)])
        for (int fIndex : replaceFroms) {
            for (int tIndex : replaceTos) {
                int current;
                if (fIndex < tIndex) {
                    current = tIndex - fIndex;
                } else if (fIndex > tIndex) {
                    current = oldLen - fIndex + tIndex;
                } else {
                    current = 0;
                }
                if (current > max) {
                    replaceFrom = fIndex;
                    replaceTo = tIndex;
                    max = current;
                }
            }
        }

        /* replace relevant vertices */
        float[][] ret = new float[2][];

        if (replaceFrom < replaceTo) {
            int newLen = oldLen - (replaceTo - replaceFrom) + 1;
            if (newLen <= oldLen / 2) {
                return polygon;
            }
            ret[0] = new float[newLen];
            ret[1] = new float[newLen];
            for (int i = 0; i < replaceFrom; i++) {
                ret[0][i] = x[i];
                ret[1][i] = z[i];
            }
            ret[0][replaceFrom] = x0;
            ret[1][replaceFrom] = z0;
            for (int i = replaceTo, j = replaceFrom + 1; i < oldLen; i++, j++) {
                ret[0][j] = x[i];
                ret[1][j] = z[i];
            }
        } else if (replaceTo < replaceFrom) {
            int newLen = replaceFrom - replaceTo + 1;
            if (newLen <= oldLen / 2) {
                return polygon;
            }
            ret[0] = new float[newLen];
            ret[1] = new float[newLen];
            for (int i = replaceTo, j = 0; i < replaceFrom; i++, j++) {
                ret[0][j] = x[i];
                ret[1][j] = z[i];
            }
            ret[0][replaceFrom - replaceTo] = x0;
            ret[1][replaceFrom - replaceTo] = z0;
        } else {

            int newLen = oldLen + 1;
            ret[0] = new float[newLen];
            ret[1] = new float[newLen];
            for (int i = 0; i < replaceFrom; i++) {
                ret[0][i] = x[i];
                ret[1][i] = z[i];
            }
            ret[0][replaceFrom] = x0;
            ret[1][replaceFrom] = z0;
            for (int i = replaceFrom; i < oldLen; i++) {
                ret[0][i + 1] = x[i];
                ret[1][i + 1] = z[i];
            }
        }
        return ret;
    }

    /**
     * @param x0      x coordinate of point outside the polygon
     * @param z0      z coordinate of point outside the polygon
     * @param xi      x coordinate of a polygon's vertex
     * @param zi      z coordinate of a polygon's vertex
     * @param polygon the polygon (in format [x1, z1, x2, z2,...])
     * @return true iff the line connecting (x0, z0) and (xi, zi) intersects the polygon
     */
    private static boolean doesConnectingLineIntersectPolygon(float x0, float z0, float xi, float zi,
                                                              float[] polygon) {

        TwoDLine l = TwoDLine.Create_From_Two_Points(x0, z0, xi, zi);
        float vertexDist = TwoDLine.Distance_Between_Points(x0, z0, xi, zi);

        ArrayList<TwoDLine> intersectionSides = l.find_all_lines(polygon);
        for (TwoDLine side : intersectionSides) {
            float[] intersection = l.Find_InterSection(side);
            float dist = TwoDLine.Distance_Between_Points(x0, z0,
                    intersection[0], intersection[1]);
            if (dist < vertexDist) {
                return true;
            }

        }

        return false;
    }

    static Pose project_pose_to_plane(Plane plane, Pose pose) {
        float[] pose_translation = pose.getTranslation();
        float xPose = pose_translation[0];
        float yPose = pose_translation[1];
        float zPose = pose_translation[2];
        Pose centerPose = plane.getCenterPose();
        float[] centerPose_translation = centerPose.getTranslation();
        float xCP = centerPose_translation[0];
        float yCP = centerPose_translation[1];
        float zCP = centerPose_translation[2];
        float[] norm = centerPose.getRotationQuaternion();
        float a = norm[0];
        float b = norm[1];
        float c = norm[2];
        float vector_length = (a * (xCP - xPose) + b * (yCP - yPose) + c * (zCP - zPose)) / (a * a + b * b + c * c);
        float[] target_translation = new float[3];
        target_translation[0] = xPose;// + vector_length * a;
        target_translation[1] = yCP;//yPose + vector_length * b;
        target_translation[2] = zPose;// + vector_length * c;
        return new Pose(target_translation, pose.getRotationQuaternion());

        /**
         *  projects a pose on to a given plane
         *
         * @param plane a Plane
         * @param pose A pose
         * @return A Pose object which is the projection of the pose on the Plane (along the
         * Plane's norma)
         */
    }

    private static float[] doubleArrayToSingle(float[][] xzSeparated) {
        float[] retInXZForm = new float[2 * xzSeparated[0].length];
        for (int i = 0; i < xzSeparated[0].length; i++) {
            retInXZForm[2 * i] = xzSeparated[0][i];
            retInXZForm[2 * i + 1] = xzSeparated[1][i];
        }

        return retInXZForm;
    }

    static float[] Convert_Point_From_Reality_to_Plane_Given_Angle(Pose point, Plane plane) {
        float[] XAxis = plane.getCenterPose().getXAxis();
        float[] ZAxis = plane.getCenterPose().getZAxis();
        float xp = point.tx() * XAxis[0] + point.tz() * XAxis[2];
        float zp = point.tx() * ZAxis[0] + point.tz() * ZAxis[2];
        /*float angle = (float) (Math.atan2(plane.getCenterPose().getXAxis()[2],plane.getCenterPose().getXAxis()[0]));
        float xp = (float) (point.tx() * Math.cos(angle) - point.tz() * Math.sin(angle)) - plane.getCenterPose().getTranslation()[0];
        float zp = (float) (-point.tx() * Math.sin(angle) - point.tz() * (Math.cos(angle))) - plane.getCenterPose().getTranslation()[2];*/
        return new float[]{xp, zp};
    }



    static float findFinalDistance(float[] polygon, float[][] objectsInside){
        return Math.min(findFinalDistance(polygon, objectsInside, 'X', false, null),findFinalDistance(polygon, objectsInside, 'Z', false, null));
    }

    private static float findFinalDistance(float[] polygon, float[][] objectsInside, char axis, boolean meter, TwoDLine meterAxis)
    //each object is sorted as [minX, minZ, maxX, maxZ] in the float-arrays array
    {
        //parameters for the resolution
        double EDGE_FACTOR = 0.5; //which distance from edges we stop checking
        if (meter) {
            EDGE_FACTOR = 0;
            if (meterAxis.getA() == 0)
                axis = 'Z';
            else if (-meterAxis.getB()/meterAxis.getA() > 1 || -meterAxis.getB()/meterAxis.getA() < -1)
                axis = 'Z';
            else
                axis = 'X';
        }
        double STEP_FACTOR = 0.1; //the stepsize we go in each axis
        int MAX_POINTS = objectsInside.length; // the maximum amount of objects we estimate to be in one axis
        float maxX = 0, minX = 0, maxZ = 0, minZ = 0;
        //find the minimal and maximal x and z coordinates in the polygon
        for (int i = 0; i < polygon.length/2; i++) {
            if (polygon[2 * i] < minX)
                minX = polygon[2 * i];
            else if (polygon[2 * i] > maxX)
                maxX = polygon[2 * i];
            if (polygon[2 * i + 1] < minZ)
                minZ = polygon[2 * i + 1];
            else if (polygon[2 * i + 1] > maxZ)
                maxZ = polygon[2 * i + 1];
        }

        float minimalMaxDistance = 999999999f;

        for (double i = minX + EDGE_FACTOR; i < maxX - EDGE_FACTOR; i += STEP_FACTOR) {
            float a, b, c;
            if (axis == 'X') {
                // check if we need to check the X or Z axis
                a = 0;
                b = 1;
                c = (float) (-1 * i);
            } else {
                a = 1;
                b = 0;
                c = (float) (-1 * i);
            }
            TwoDLine Axis = new TwoDLine(a, b, c); // define a line of the required orientation
            TwoDLine[] lines = Axis.find_lines(polygon); //find the intersections of this line with
            //the polygon
            float[] intersection1 = Axis.Find_InterSection(lines[0]);
            float[] intersection2 = Axis.Find_InterSection(lines[1]);
            if(intersection1[2] != 1 || intersection2[2] != 1){
                continue;
            }
            float[] onDiameter = new float[MAX_POINTS * 4 + 4];//the array in which we put the points which are on our Axis
            int added_counter = 2; // counts how many coordinates we added (pointsX2)
            onDiameter[0] = intersection1[0];
            onDiameter[1] = intersection1[1];
            onDiameter[onDiameter.length - 2] = intersection2[0];
            onDiameter[onDiameter.length - 1] = intersection2[1];
            for (int j = 0; j < objectsInside.length; j++)
            //find the intersections with other objects of this longitude
            {
                float[] object = objectsInside[j]; // float array at the size of 8 - 2 for each corner
                float objectMinXX = object[0];
                float objectMinXZ = object[1];
                float objectMinZX = object[2];
                float objectMinZZ = object[3];
                float objectMaxXX = object[4];
                float objectMaxXZ = object[5];
                float objectMaxZX = object[6];
                float objectMaxZZ = object[7];
                if (Axis.is_intersection_in_between_points(objectMinXX, objectMinXZ,
                        objectMinZX, objectMinZZ))// checks if the intersection is on the side of the object
                {
                    float[] intersection = Axis.Find_InterSection
                            (TwoDLine.Create_From_Two_Points(objectMinXX, objectMinXZ,
                                    objectMinZX, objectMinZZ));
                    onDiameter[added_counter] = intersection[0];
                    onDiameter[added_counter + 1] = intersection[1];
                    added_counter += 2;
                }
                if (Axis.is_intersection_in_between_points(objectMinXX, objectMinXZ,
                        objectMaxZX, objectMaxZZ)) {
                    float[] intersection = Axis.Find_InterSection
                            (TwoDLine.Create_From_Two_Points(objectMinXX, objectMinXZ,
                                    objectMaxZX, objectMaxZZ));
                    onDiameter[added_counter] = intersection[0];
                    onDiameter[added_counter + 1] = intersection[1];
                    added_counter += 2;
                }
                if (Axis.is_intersection_in_between_points(objectMaxXX, objectMaxXZ,
                        objectMinZX, objectMinZZ)) {
                    float[] intersection = Axis.Find_InterSection
                            (TwoDLine.Create_From_Two_Points(objectMaxXX, objectMaxXZ,
                                    objectMinZX, objectMinZZ));
                    onDiameter[added_counter] = intersection[0];
                    onDiameter[added_counter + 1] = intersection[1];
                    added_counter += 2;
                }
                if (Axis.is_intersection_in_between_points(objectMaxXX, objectMaxXZ,
                        objectMaxZX, objectMaxZZ)) {
                    float[] intersection = Axis.Find_InterSection
                            (TwoDLine.Create_From_Two_Points(objectMaxXX, objectMaxXZ,
                                    objectMaxZX, objectMaxZZ));
                    onDiameter[added_counter] = intersection[0];
                    onDiameter[added_counter + 1] = intersection[1];
                    added_counter += 2;
                }

            }
            float max_distance = 0;
            for (int k = 0; k < onDiameter.length / 4; k++)
                // check the maximum passable length between objects
                if (TwoDLine.Distance_Between_Points(onDiameter[4 * k], onDiameter[4 * k + 1],
                        onDiameter[4 * k + 2], onDiameter[4 * k + 3]) > max_distance)
                    max_distance = TwoDLine.Distance_Between_Points(onDiameter[4 * k],
                            onDiameter[4 * k + 1], onDiameter[4 * k + 2], onDiameter[4 * k + 3]);

            if (max_distance < minimalMaxDistance) // we need to find the minimal maximal size to know in which distance we can pass
                minimalMaxDistance = max_distance;
        }
        return minimalMaxDistance;
    }

    static float[][] four_points_of_all_objects(Plane plane, ArrayList<ArrayList<float[]>> all_object_points) {
        float[][] arr = new float[all_object_points.size()][4];
        for (int i = 0; i < all_object_points.size(); i++) {
            float[] obj_arr = four_points_of_object(plane, all_object_points.get(i));
            int[] indices = new int[]{1, 2, 4, 5, 7, 8, 10, 11};
            float[] obj_a = new float[8];
            for (int j = 0; j < 8; j++)
                obj_a[j] = obj_arr[indices[j]];
            arr[i] = obj_a;
        }
        return arr;
    }

    private static float[] four_points_of_object(Plane plane, ArrayList<float[]> object_points)
    {
        float[] min_z_p = new float[3], max_z_p=new float[3], min_x_p = new float[3], max_x_p = new float[3];
        for(float[] p: object_points)
        {
            Pose pose = new Pose(p,new float[]{0,0,0,0});
            float[] point_in_plane_system = Convert_Point_From_Reality_to_Plane_Given_Angle(pose, plane);
            float x = point_in_plane_system[0], z = point_in_plane_system[1];
            if (x>max_x_p[0])
            {
                max_x_p[0] = x;
                max_x_p[1] = z;
                max_x_p[2] = convert_bool_to_float(plane.isPoseInPolygon(pose));
            }
            if (x<min_x_p[0])
            {
                min_x_p[0]=x;
                min_x_p[1]=z;
                min_x_p[2] = convert_bool_to_float(plane.isPoseInPolygon(pose));
            }
            if (z>max_z_p[1])
            {
                max_z_p[0]=x;
                max_z_p[1]=z;
                max_z_p[2] = convert_bool_to_float(plane.isPoseInPolygon(pose));
            }
            if (z<min_z_p[1])
            {
                min_z_p[0]=x;
                min_z_p[1]=z;
                min_z_p[2] = convert_bool_to_float(plane.isPoseInPolygon(pose));
            }
        }
        float [] to_ret = new float[13];
        float points_inside = min_x_p[2]+max_x_p[2]+max_z_p[2]+min_z_p[2];
        return new float[]{points_inside,min_x_p[0],min_x_p[1],min_x_p[2],min_z_p[0],min_z_p[1],min_z_p[2],max_x_p[0],max_x_p[1],max_x_p[2],max_z_p[0],max_z_p[1],max_z_p[2]};
    }

    private static float convert_bool_to_float(boolean bool) {
        if (bool)
            return 1;
        else
            return 0;
    }

    float[] createPlane (PointCloud cloud, Plane plane, float[] polygon, ArrayList<ArrayList<float[]>> objects)
    {
        float[][] objectsInside = new float[0][0];
        for (ArrayList<float[]> object : objects)
        {
            boolean allOut = true;
            boolean allIn = true;
            for (float[] point: object) {
                float[] poseCoordiantes = new float[3];
                poseCoordiantes[0] = point[0];
                poseCoordiantes[1] = point[1];
                poseCoordiantes[2] = point[2];
                float[] fakeQuaternions = new float[4];
                fakeQuaternions[0] = 0;
                fakeQuaternions[1] = 0;
                fakeQuaternions[2] = 1;
                fakeQuaternions[3] = 0;
                Pose pose = new Pose(poseCoordiantes,fakeQuaternions);
                if (plane.isPoseInPolygon(pose))
                    allOut = false;
                else
                    allIn = false;
            }
            if (allIn || allOut) {
                objects.remove(object);
            }
            else
            {
                float[] real_object = four_points_of_object(plane, object);
                float[] pointsIn = new float[2* (int)real_object[0]];
                int inCounter = 0;
                int outCounter = 0;
                float[] pointsOut = new float[8-(2* (int)real_object[0])];
                for (int i=1; i< real_object.length; i+=3)
                {
                    if (real_object[i+2] == 0)
                    {
                        pointsIn[inCounter] = real_object[i];
                        pointsIn[inCounter+1] = real_object[i+1];
                        inCounter += 2;
                    }
                    else
                    {
                        pointsOut[outCounter] = real_object[i];
                        pointsOut[outCounter+1] = real_object[i+1];
                        outCounter += 2;
                    }
                }
                polygon = cut_plane_points_inside(polygon, pointsIn, pointsOut);
            }
        }
        return polygon;
    }

    private float[] cut_plane_points_inside(float[] plane, float[] pointsIn, float[] pointsOut) {
        float[] temp_pointsIn = new float[4];
        float[] temp_pointsOut = new float[4];
        switch (pointsIn.length) {
            case 0:
                return plane;
            case 2:
                for (int i = 0; i < 4; i++) {
                    temp_pointsIn[i] = temp_pointsIn[i % 2];
                }
                temp_pointsOut[0] = pointsOut[0];
                temp_pointsOut[1] = pointsOut[1];
                temp_pointsOut[2] = pointsOut[4];
                temp_pointsOut[3] = pointsOut[5];
            case 4:
                temp_pointsIn = pointsIn;
                temp_pointsOut = pointsOut;
            case 6:
                for (int i = 0; i < 4; i++) {
                    temp_pointsOut[i] = temp_pointsOut[i % 2];
                }
                temp_pointsIn[0] = pointsIn[0];
                temp_pointsIn[1] = pointsIn[1];
                temp_pointsIn[2] = pointsIn[4];
                temp_pointsIn[3] = pointsIn[5];
            case 8:
                return plane;
        }
        float[] edge_points = find_plane_and_object_edge_points(plane, temp_pointsIn, temp_pointsOut);
        return change_plane(plane, edge_points, pointsIn);
    }

    private float[] find_plane_and_object_edge_points(float[] plane, float[] pointsIn, float[] pointsOut) {
        // points in and out match each other (right-left), must give only 2 points in and
        // 2 points out
        float[] current_points = new float[8];
        for (int i = 0; i < current_points.length; i++) {
            current_points[i] = 0;
        }
        float[][] sorted = sort_plane(plane);
        float minimal_distance1 = 100000;
        float minimal_distance2 = 100000;
        float distance;
        for (int i = 0; i < sorted[0].length; i++) {
            distance = TwoDLine.Distance_Between_Points(sorted[0][i], sorted[1][i], pointsIn[0], pointsIn[1]) +
                    TwoDLine.Distance_Between_Points(sorted[0][i], sorted[1][i], pointsOut[0], pointsOut[1]);
            if (distance < minimal_distance2) {
                if (distance < minimal_distance1) {
                    minimal_distance2 = minimal_distance1;
                    minimal_distance1 = distance;
                    current_points[2] = current_points[0];
                    current_points[3] = current_points[1];
                    current_points[0] = sorted[0][i];
                    current_points[1] = sorted[1][i];
                } else {
                    minimal_distance2 = distance;
                    current_points[2] = sorted[0][i];
                    current_points[3] = sorted[1][i];
                }
            }
        }
        minimal_distance1 = 100000;
        minimal_distance2 = 100000;
        for (int i = 0; i < sorted[0].length; i++) {
            distance = TwoDLine.Distance_Between_Points(sorted[0][i], sorted[1][i], pointsIn[2], pointsIn[3]) +
                    TwoDLine.Distance_Between_Points(sorted[0][i], sorted[1][i], pointsOut[2], pointsOut[3]);
            if (distance < minimal_distance2) {
                if (distance < minimal_distance1) {
                    minimal_distance2 = minimal_distance1;
                    minimal_distance1 = distance;
                    current_points[6] = current_points[4];
                    current_points[7] = current_points[5];
                    current_points[4] = sorted[0][i];
                    current_points[5] = sorted[1][i];
                } else {
                    minimal_distance2 = distance;
                    current_points[6] = sorted[4][i];
                    current_points[7] = sorted[5][i];
                }
            }
        }
        /*TwoDLine lineOne = TwoDLine.Create_From_Two_Points(pointsIn[0], pointsIn[1], pointsOut[0], pointsOut[1]);
        TwoDLine lineTwo = TwoDLine.Create_From_Two_Points(pointsIn[2], pointsIn[3], pointsOut[2], pointsOut[3]);
        TwoDLine lineOneInter = TwoDLine.Create_From_Two_Points(current_points[0], current_points[1], current_points[2], current_points[3]);
        TwoDLine lineTwoInter = TwoDLine.Create_From_Two_Points(current_points[4], current_points[5], current_points[6], current_points[7]);
        float[] intersec1 = lineOne.Find_InterSection(lineOneInter);
        float[] intersec2 = lineTwo.Find_InterSection(lineTwoInter);
        float[] intersections = new float[4];
        intersections[0] = intersec1[0];
        intersections[1] = intersec1[1];
        intersections[2] = intersec2[0];
        intersections[3] = intersec2[1];*/
        return current_points;
    }

    private float[][] sort_plane(float[] plane) {
        float[][] sorted = new float[2][plane.length / 2];
        float[] planesX = new float[plane.length / 2];
        float[] planesZ = new float[plane.length / 2];
        for (int i = 0; i < plane.length; i++) {
            sorted[0][i] = plane[2 * i];
            sorted[1][i] = plane[2 * i + 1];
        }
        return sorted;
    }

    private float[] change_plane(float[] plane, float[] sides, float[] new_points) {
        for (int i = 0; i < plane.length - 1; i++) {
            if (plane[i] == sides[0] && plane[i + 1] == sides[1]) {
                boolean done = false;
                for (int j = 0; j < plane.length && !done; j++) {
                    if (plane[j] == sides[6] && plane[j + 1] == sides[7]) {
                        done = true;
                        if (i < j) {
                            float[] new_plane = new float[plane.length + i / 2 - j / 2 + 2];
                            System.arraycopy(plane, 0, new_plane, 0, i + 2);
                            for (int h = 0; h < new_points.length; h++) {
                                new_plane[i + 2 + h] = new_points[h];
                            }
                            for (int g = j; g < plane.length; g++) {
                                new_plane[g - j + i + new_points.length] = plane[g];
                            }
                            return new_plane;
                        }
                    }
                }

            }
        }
        return null;
    }


    float merge_plane_width (PointCloud cloud, Plane plane, float[] polygon, ArrayList<ArrayList<float[]>> objects) {
        float[][] objectsInside = new float[0][0];
        for (ArrayList<float[]> object : objects)
        {
            boolean allOut = true;
            boolean allIn = true;
            for (float[] point: object) {
                float[] poseCoordiantes = new float[3];
                poseCoordiantes[0] = point[0];
                poseCoordiantes[1] = point[1];
                poseCoordiantes[2] = point[2];
                float[] fakeQuaternions = new float[4];
                fakeQuaternions[0] = 0;
                fakeQuaternions[1] = 0;
                fakeQuaternions[2] = 1;
                fakeQuaternions[3] = 0;
                Pose pose = new Pose(poseCoordiantes,fakeQuaternions);
                if (plane.isPoseInPolygon(pose))
                    allOut = false;
                else
                    allIn = false;
            }
            if (allIn || allOut) {
                objects.remove(object);
            }
            else
            {
                float[] real_object = four_points_of_object(plane, object);
                float[] pointsIn = new float[2* (int)real_object[0]];
                int inCounter = 0;
                int outCounter = 0;
                float[] pointsOut = new float[8-(2* (int)real_object[0])];
                for (int i=1; i< real_object.length; i+=3)
                {
                    if (real_object[i+2] == 0)
                    {
                        pointsIn[inCounter] = real_object[i];
                        pointsIn[inCounter+1] = real_object[i+1];
                        inCounter += 2;
                    }
                    else
                    {
                        pointsOut[outCounter] = real_object[i];
                        pointsOut[outCounter+1] = real_object[i+1];
                        outCounter += 2;
                    }
                }
                polygon = cut_plane_points_inside(polygon, pointsIn, pointsOut);
            }
        }
        float Xdistance = findFinalDistance(polygon,objectsInside,'X', false, new TwoDLine(1,1,1));
        float Zdistance = findFinalDistance(polygon, objectsInside, 'Z', false, new TwoDLine(1,1,1));
        return Math.min(Xdistance, Zdistance);
    }

    static float meter_ahead (Plane plane, float[] polygon, float[][] objectsInside, Camera camera){
        Pose cameraPose = camera.getPose();
        Pose projected = project_pose_to_plane(plane, cameraPose);
        float[] cameraPoseOnPlane = Convert_Point_From_Reality_to_Plane_Given_Angle(projected,plane);
        float [] newTranslation = {cameraPose.getTranslation()[0], cameraPose.getTranslation()[1] + 1,
                cameraPose.getTranslation()[2]};
        float [] newQuaternions = {cameraPose.getRotationQuaternion()[0], cameraPose.getRotationQuaternion()[1],
                cameraPose.getRotationQuaternion()[2], cameraPose.getRotationQuaternion()[3]};
        Pose meterAhead = new Pose (newTranslation, newQuaternions);
        Pose meterProjected = project_pose_to_plane(plane, meterAhead);
        float[] meterAheadOnPlane = Convert_Point_From_Reality_to_Plane_Given_Angle(meterProjected, plane);
        TwoDLine connection = TwoDLine.Create_From_Two_Points(cameraPoseOnPlane[0], cameraPoseOnPlane[1],
                meterAheadOnPlane[0], meterAheadOnPlane[1]);
        TwoDLine diagonaCamera = connection.Vertical(cameraPoseOnPlane[0], cameraPoseOnPlane[1]);
        TwoDLine diagonalAhead = connection.Vertical(meterAheadOnPlane[0], meterAheadOnPlane[1]);
        int [] cutInexes = new int[4];
        boolean foundOneCamera = false;
        boolean foundOneMeter = false;
        for (int i = 0; i < polygon.length - 2; i+=2){
            // Finding he indexes in which to cut the old polygon
            if (diagonaCamera.Distance_To_Point_Not_Abs(polygon[i], polygon[i+1])/
                    diagonaCamera.Distance_To_Point_Not_Abs(polygon[i+2],polygon[i+3]) < 0){
                // May need checking in which index to cut - didn't do it accuratly
                if (!foundOneCamera) {
                    cutInexes[0] = i;
                    foundOneCamera = true;
                }
                else
                    cutInexes[1] = i;
            }

            else if (diagonalAhead.Distance_To_Point_Not_Abs(polygon[i], polygon[i+1])/
                    diagonalAhead.Distance_To_Point_Not_Abs(polygon[i+2],polygon[i+3]) < 0){
                // May need checking in which index to cut - didn't do it accuratly
                if (!foundOneMeter) {
                    cutInexes[2] = i;
                    foundOneMeter = true;
                }
                else
                    cutInexes[3] = i;
            }
        }
        float[] newPolygon = new float [polygon.length];
        int newPolygonCounter = 0;
                for (int j = 0; j < polygon.length * 2; j += 2){
                    if(j % polygon.length == cutInexes[1]) {
                        int tempJ = j;
                        int temp = polygon.length;
                        for (int k = j; k < polygon.length * 2; k++) {
                            if (k % polygon.length == cutInexes[2] || k % polygon.length == cutInexes[3]) { // found the other line
                                while (tempJ % polygon.length != cutInexes[3] && tempJ % polygon.length != cutInexes[2]) {//add from one line to the other
                                    newPolygon[newPolygonCounter] = polygon[tempJ % polygon.length];
                                    newPolygonCounter += 1;
                                    tempJ++;
                                }
                                newPolygon[newPolygonCounter] = meterAheadOnPlane[0];
                                newPolygon[newPolygonCounter + 1] = meterAheadOnPlane[1];
                                newPolygonCounter += 2;
                                temp = k + 1;
                                while (temp % polygon.length != cutInexes[0]) {
                                    newPolygon[newPolygonCounter] = polygon[temp % polygon.length];
                                    newPolygonCounter++;
                                    temp++;
                                }
                                newPolygon[newPolygonCounter] = cameraPoseOnPlane[0];
                                newPolygon[newPolygonCounter + 1] = cameraPoseOnPlane[1];
                                newPolygonCounter += 2;
                                break;
                            } else if (k == cutInexes[0]) {
                                newPolygon[newPolygonCounter] = cameraPoseOnPlane[0];
                                newPolygon[newPolygonCounter + 1] = cameraPoseOnPlane[1];
                                newPolygonCounter += 2;
                                k += 2;
                                while (k % polygon.length != cutInexes[2] && k % polygon.length != cutInexes[3]) {
                                    newPolygon[newPolygonCounter] = polygon[k % polygon.length];
                                    newPolygonCounter++;
                                    k++;
                                }
                                newPolygon[newPolygonCounter] = meterAheadOnPlane[0];
                                newPolygon[newPolygonCounter + 1] = meterAheadOnPlane[1];
                                newPolygonCounter += 2;
                                while (k % polygon.length != cutInexes[2] && k % polygon.length != cutInexes[3])
                                    k++;
                                while (k % polygon.length != cutInexes[0]) {
                                    newPolygon[newPolygonCounter] = polygon[k % polygon.length];
                                    k++;
                                }
                                break;
                            }
                        }
                    }
                }




        return findFinalDistance(newPolygon, objectsInside, 'X', true, diagonaCamera);
        //return findFinalDistance(newPolygon, objectsInside, diagonaCamera);
    }
}