package com.google.ar.core.examples.java.helloar;

import android.view.MotionEvent;

import com.google.ar.core.Anchor;
import com.google.ar.core.Camera;
import com.google.ar.core.Frame;
import com.google.ar.core.HitResult;
import com.google.ar.core.Plane;
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
    public static float distanceBetweenPoses(Pose pose1, Pose pose2) {
        float dx = pose1.tx() - pose2.tx();
        float dy = pose1.ty() - pose2.ty();
        float dz = pose1.tz() - pose2.tz();

        // Compute the straight-line distance.
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    public static float horizontalDistanceBetweenPoses(Pose pose1, Pose pose2) {
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
    public static float distanceBetweenPoses(float[] pose1, float[] pose2) {
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
    public static ArrayList<ArrayList<float[]>> getObjects(float[] points, Plane floor) {
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
        float distanceThreshold = 0.3f;
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
                    if (checked_indexes.contains(i))
                        continue;
                    if (distanceBetweenPoses(object.get(point_index), allPoints.get(i)) < distanceThreshold) {
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
    public static float pixelToDistance(float[] pixel, Frame frame) {
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
    public static Plane getFloor(ArrayList<Plane> planes, Camera camera) {
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
    public static float find_width(Plane floor) {
        return find_width(floor.getPolygon().array());
    }

    private static float find_width(float[] points) {
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
    public static ArrayList<float[]> findExtensionPoints(Plane floor, float[] points) {
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

    public static float[][] polygon_to_xz(float[] points) {
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
    public static float[] extendFloor(float[] floorPolygon, ArrayList<float[]> extensionPoints, float x_extent, float z_extent) {

        /*
        float[][] xz = polygon_to_xz(floorPolygon);
        float[] x = xz[0];
        float[] z = xz[1];
        */

        float[][] ret = polygon_to_xz(floorPolygon);

        for (float[] point : extensionPoints) {
            ret = extendSinglePoint(ret, point[0], point[1], doubleArrayToSingle(ret),x_extent,z_extent);
        }

        float[] retAsSingleArray = doubleArrayToSingle(ret);
        return retAsSingleArray;
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

    public static Pose project_pose_to_plane(Plane plane, Pose pose) {
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

    public static float[] Convert_Point_From_Reality_to_Plane_Given_Angle(Pose point, Plane plane) {
        float[] XAxis = plane.getCenterPose().getXAxis();
        float[] ZAxis = plane.getCenterPose().getZAxis();
        float xp = point.tx() * XAxis[0] + point.tz() * XAxis[2];
        float zp = point.tx() * ZAxis[0] + point.tz() * ZAxis[2];
        /*float angle = (float) (Math.atan2(plane.getCenterPose().getXAxis()[2],plane.getCenterPose().getXAxis()[0]));
        float xp = (float) (point.tx() * Math.cos(angle) - point.tz() * Math.sin(angle)) - plane.getCenterPose().getTranslation()[0];
        float zp = (float) (-point.tx() * Math.sin(angle) - point.tz() * (Math.cos(angle))) - plane.getCenterPose().getTranslation()[2];*/
        return new float[]{xp, zp};
    }
}