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
                    if(checked_indexes.contains(i))
                        continue;
                    if(distanceBetweenPoses(object.get(point_index),allPoints.get(i)) < distanceThreshold){
                        object.add(allPoints.get(i));
                        checked_indexes.add(i);
                    }
                }
            }
        }
        return objects;
        /*for (float[] currPoint : allPoints) {
            // check that the point isn't on the floor
            // threshold for distance from points in already found objects
            //TODO can do better in classifying objects using using smarter clustering algorithms
            boolean foundObject = false;
            for (ArrayList<float[]> object : objects) {
                if (foundObject) {
                    break;
                }
                int index = binarySearch(currPoint, object);
                if (index != object.size()) {
                    if (distanceBetweenPoses(currPoint, object.get(index)) < distanceThreshold) {
                        object.add(index, currPoint);
                        foundObject = true;
                    }
                } else {
                    if (distanceBetweenPoses(currPoint, object.get(index - 1)) < distanceThreshold) {
                        object.add(index, currPoint);
                        foundObject = true;
                    }
                }
                if (index < object.size() - 1 && distanceBetweenPoses(currPoint, object.get(index + 1)) < distanceThreshold) {
                    object.add(index, currPoint);
                    foundObject = true;
                }
            }
            // new object detected
            if (!foundObject) {
                ArrayList<float[]> newObject = new ArrayList<float[]>();
                newObject.add(currPoint);
                objects.add(newObject);
            }
        }
        return objects;*/
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
}
