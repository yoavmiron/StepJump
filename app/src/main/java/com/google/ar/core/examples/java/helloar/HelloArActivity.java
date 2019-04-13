/*
 * Copyright 2017 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.google.ar.core.examples.java.helloar;

import java.lang.Math;

import com.google.ar.core.examples.java.helloar.TwoDLine;

import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MotionEvent;
import android.widget.ArrayAdapter;
import android.widget.Toast;

import com.google.ar.core.Anchor;
import com.google.ar.core.ArCoreApk;
import com.google.ar.core.Camera;
import com.google.ar.core.Frame;
import com.google.ar.core.HitResult;
import com.google.ar.core.Plane;
import com.google.ar.core.Point;
import com.google.ar.core.Point.OrientationMode;
import com.google.ar.core.PointCloud;
import com.google.ar.core.Pose;
import com.google.ar.core.Session;
import com.google.ar.core.Trackable;
import com.google.ar.core.TrackingState;
import com.google.ar.core.examples.java.common.helpers.CameraPermissionHelper;
import com.google.ar.core.examples.java.common.helpers.DisplayRotationHelper;
import com.google.ar.core.examples.java.common.helpers.FullScreenHelper;
import com.google.ar.core.examples.java.common.helpers.SnackbarHelper;
import com.google.ar.core.examples.java.common.helpers.TapHelper;
import com.google.ar.core.examples.java.common.rendering.BackgroundRenderer;
import com.google.ar.core.examples.java.common.rendering.ObjectRenderer;
import com.google.ar.core.examples.java.common.rendering.ObjectRenderer.BlendMode;
import com.google.ar.core.examples.java.common.rendering.PlaneRenderer;
import com.google.ar.core.examples.java.common.rendering.PointCloudRenderer;
import com.google.ar.core.exceptions.CameraNotAvailableException;
import com.google.ar.core.exceptions.UnavailableApkTooOldException;
import com.google.ar.core.exceptions.UnavailableArcoreNotInstalledException;
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException;
import com.google.ar.core.exceptions.UnavailableSdkTooOldException;
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException;

import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Comparator;


import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import static java.lang.Math.max;

/**
 * This is a simple example that shows how to create an augmented reality (AR) application using the
 * ARCore API. The application will display any detected planes and will allow the user to tap on a
 * plane to place a 3d model of the Android robot.
 */
public class HelloArActivity extends AppCompatActivity implements GLSurfaceView.Renderer {
    private static final String TAG = HelloArActivity.class.getSimpleName();

    // Rendering. The Renderers are created here, and initialized when the GL surface is created.
    private GLSurfaceView surfaceView;

    private boolean installRequested;

    private Session session;
    private final SnackbarHelper messageSnackbarHelper = new SnackbarHelper();
    private DisplayRotationHelper displayRotationHelper;
    private TapHelper tapHelper;

    private final BackgroundRenderer backgroundRenderer = new BackgroundRenderer();
    private final ObjectRenderer virtualObject = new ObjectRenderer();
    private final ObjectRenderer virtualObjectShadow = new ObjectRenderer();
    private final PlaneRenderer planeRenderer = new PlaneRenderer();
    private final PointCloudRenderer pointCloudRenderer = new PointCloudRenderer();

    // Temporary matrix allocated here to reduce number of allocations for each frame.
    private final float[] anchorMatrix = new float[16];
    private static final float[] DEFAULT_COLOR = new float[]{0f, 0f, 0f, 0f};

    // Anchors created from taps used for object placing with a given color.
    private static class ColoredAnchor {
        public final Anchor anchor;
        public final float[] color;

        public ColoredAnchor(Anchor a, float[] color4f) {
            this.anchor = a;
            this.color = color4f;
        }
    }

    private final ArrayList<ColoredAnchor> anchors = new ArrayList<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        surfaceView = findViewById(R.id.surfaceview);
        displayRotationHelper = new DisplayRotationHelper(/*context=*/ this);

        // Set up tap listener.
        tapHelper = new TapHelper(/*context=*/ this);
        surfaceView.setOnTouchListener(tapHelper);

        // Set up renderer.
        surfaceView.setPreserveEGLContextOnPause(true);
        surfaceView.setEGLContextClientVersion(2);
        surfaceView.setEGLConfigChooser(8, 8, 8, 8, 16, 0); // Alpha used for plane blending.
        surfaceView.setRenderer(this);
        surfaceView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);

        installRequested = false;
    }

    @Override
    protected void onResume() {
        super.onResume();

        if (session == null) {
            Exception exception = null;
            String message = null;
            try {
                switch (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
                    case INSTALL_REQUESTED:
                        installRequested = true;
                        return;
                    case INSTALLED:
                        break;
                }

                // ARCore requires camera permissions to operate. If we did not yet obtain runtime
                // permission on Android M and above, now is a good time to ask the user for it.
                if (!CameraPermissionHelper.hasCameraPermission(this)) {
                    CameraPermissionHelper.requestCameraPermission(this);
                    return;
                }

                // Create the session.
                session = new Session(/* context= */ this);

            } catch (UnavailableArcoreNotInstalledException
                    | UnavailableUserDeclinedInstallationException e) {
                message = "Please install ARCore";
                exception = e;
            } catch (UnavailableApkTooOldException e) {
                message = "Please update ARCore";
                exception = e;
            } catch (UnavailableSdkTooOldException e) {
                message = "Please update this app";
                exception = e;
            } catch (UnavailableDeviceNotCompatibleException e) {
                message = "This device does not support AR";
                exception = e;
            } catch (Exception e) {
                message = "Failed to create AR session";
                exception = e;
            }

            if (message != null) {
                messageSnackbarHelper.showError(this, message);
                Log.e(TAG, "Exception creating session", exception);
                return;
            }
        }

        // Note that order matters - see the note in onPause(), the reverse applies here.
        try {
            session.resume();
        } catch (CameraNotAvailableException e) {
            // In some cases (such as another camera app launching) the camera may be given to
            // a different app instead. Handle this properly by showing a message and recreate the
            // session at the next iteration.
            messageSnackbarHelper.showError(this, "Camera not available. Please restart the app.");
            session = null;
            return;
        }

        surfaceView.onResume();
        displayRotationHelper.onResume();

        messageSnackbarHelper.showMessage(this, "Searching for surfaces...");
    }

    @Override
    public void onPause() {
        super.onPause();
        if (session != null) {
            // Note that the order matters - GLSurfaceView is paused first so that it does not try
            // to query the session. If Session is paused before GLSurfaceView, GLSurfaceView may
            // still call session.update() and get a SessionPausedException.
            displayRotationHelper.onPause();
            surfaceView.onPause();
            session.pause();
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
        if (!CameraPermissionHelper.hasCameraPermission(this)) {
            Toast.makeText(this, "Camera permission is needed to run this application", Toast.LENGTH_LONG)
                    .show();
            if (!CameraPermissionHelper.shouldShowRequestPermissionRationale(this)) {
                // Permission denied with checking "Do not ask again".
                CameraPermissionHelper.launchPermissionSettings(this);
            }
            finish();
        }
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        FullScreenHelper.setFullScreenOnWindowFocusChanged(this, hasFocus);
    }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

        // Prepare the rendering objects. This involves reading shaders, so may throw an IOException.
        try {
            // Create the texture and pass it to ARCore session to be filled during update().
            backgroundRenderer.createOnGlThread(/*context=*/ this);
            planeRenderer.createOnGlThread(/*context=*/ this, "models/trigrid.png");
            pointCloudRenderer.createOnGlThread(/*context=*/ this);

            virtualObject.createOnGlThread(/*context=*/ this, "models/andy.obj", "models/andy.png");
            virtualObject.setMaterialProperties(0.0f, 2.0f, 0.5f, 6.0f);

            virtualObjectShadow.createOnGlThread(
                    /*context=*/ this, "models/andy_shadow.obj", "models/andy_shadow.png");
            virtualObjectShadow.setBlendMode(BlendMode.Shadow);
            virtualObjectShadow.setMaterialProperties(1.0f, 0.0f, 0.0f, 1.0f);

        } catch (IOException e) {
            Log.e(TAG, "Failed to read an asset file", e);
        }
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        displayRotationHelper.onSurfaceChanged(width, height);
        GLES20.glViewport(0, 0, width, height);
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        // Clear screen to notify driver it should not load any pixels from previous frame.
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT | GLES20.GL_DEPTH_BUFFER_BIT);

        if (session == null) {
            return;
        }
        // Notify ARCore session that the view size changed so that the perspective matrix and
        // the video background can be properly adjusted.
        displayRotationHelper.updateSessionIfNeeded(session);
        Plane floor = null;

        try {
            session.setCameraTextureName(backgroundRenderer.getTextureId());

            // Obtain the current frame from ARSession. When the configuration is set to
            // UpdateMode.BLOCKING (it is by default), this will throttle the rendering to the
            // camera framerate.
            Frame frame = session.update();
            Camera camera = frame.getCamera();

            // Handle one tap per frame.
            handleTap(frame, camera);

            // Draw background.
            backgroundRenderer.draw(frame);

            // If not tracking, don't draw 3d objects.
            if (camera.getTrackingState() == TrackingState.PAUSED) {
                return;
            }

            // Get projection matrix.
            float[] projmtx = new float[16];
            camera.getProjectionMatrix(projmtx, 0, 0.1f, 100.0f);

            // Get camera matrix and draw.
            float[] viewmtx = new float[16];
            camera.getViewMatrix(viewmtx, 0);

            // Compute lighting from average intensity of the image.
            // The first three components are color scaling factors.
            // The last one is the average pixel intensity in gamma space.
            final float[] colorCorrectionRgba = new float[4];
            frame.getLightEstimate().getColorCorrection(colorCorrectionRgba, 0);

            // Visualize tracked points.
            PointCloud pointCloud = frame.acquirePointCloud();
            pointCloud.getPoints();
            ArrayList<Plane> ALPlanes = new ArrayList<>(session.getAllTrackables(Plane.class));
            floor = getFloor(ALPlanes,frame.getCamera());
            FloatBuffer buffer = pointCloud.getPoints();
            float[] floatBuffer = bufferToArray(buffer);
            ArrayList<ArrayList<float[]>> objects = getObjects(floatBuffer, floor);
            float meter;
            float b;
            if (floor != null){
                //meter = meter_ahead(floor, floor.getPolygon().array(), four_points_of_all_objects(floor,objects), frame.acquirePointCloud(),frame.getCamera());
                b = 5;}
            pointCloudRenderer.update(pointCloud);
            pointCloudRenderer.draw(viewmtx, projmtx);

            // Application is responsible for releasing the point cloud resources after
            // using it.

            pointCloud.release();

            // Check if we detected at least one plane. If so, hide the loading message.
            if (messageSnackbarHelper.isShowing()) {
                for (Plane plane : session.getAllTrackables(Plane.class)) {
                    if (plane.getTrackingState() == TrackingState.TRACKING) {
                        messageSnackbarHelper.hide(this);
                        break;
                    }
                }
            }

            // Visualize planes.
            planeRenderer.drawPlanes(
                    session.getAllTrackables(Plane.class), camera.getDisplayOrientedPose(), projmtx);

            // Visualize anchors created by touch.
            float scaleFactor = 1.0f;
            for (ColoredAnchor coloredAnchor : anchors) {
                if (coloredAnchor.anchor.getTrackingState() != TrackingState.TRACKING) {
                    continue;
                }
                // Get the current pose of an Anchor in world space. The Anchor pose is updated
                // during calls to session.update() as ARCore refines its estimate of the world.
                coloredAnchor.anchor.getPose().toMatrix(anchorMatrix, 0);

                // Update and draw the model and its shadow.
                virtualObject.updateModelMatrix(anchorMatrix, scaleFactor);
                virtualObjectShadow.updateModelMatrix(anchorMatrix, scaleFactor);
                virtualObject.draw(viewmtx, projmtx, colorCorrectionRgba, coloredAnchor.color);
                virtualObjectShadow.draw(viewmtx, projmtx, colorCorrectionRgba, coloredAnchor.color);
            }

        } catch (Throwable t) {
            // Avoid crashing the application due to unhandled exceptions.
            Log.e(TAG, "Exception on the OpenGL thread", t);
        }
    }

    // Handle only one tap per frame, as taps are usually low frequency compared to frame rate.
    private void handleTap(Frame frame, Camera camera) {
        MotionEvent tap = tapHelper.poll();
        if (tap != null && camera.getTrackingState() == TrackingState.TRACKING) {
            for (HitResult hit : frame.hitTest(tap)) {
                // Check if any plane was hit, and if it was hit inside the plane polygon
                Trackable trackable = hit.getTrackable();
                // Creates an anchor if a plane or an oriented point was hit.
                if ((trackable instanceof Plane
                        && ((Plane) trackable).isPoseInPolygon(hit.getHitPose())
                        && (PlaneRenderer.calculateDistanceToPlane(hit.getHitPose(), camera.getPose()) > 0))
                        || (trackable instanceof Point
                        && ((Point) trackable).getOrientationMode()
                        == OrientationMode.ESTIMATED_SURFACE_NORMAL)) {
                    // Hits are sorted by depth. Consider only closest hit on a plane or oriented point.
                    // Cap the number of objects created. This avoids overloading both the
                    // rendering system and ARCore.
                    if (anchors.size() >= 20) {
                        anchors.get(0).anchor.detach();
                        anchors.remove(0);
                    }

                    // Assign a color to the object for rendering based on the trackable type
                    // this anchor attached to. For AR_TRACKABLE_POINT, it's blue color, and
                    // for AR_TRACKABLE_PLANE, it's green color.
                    float[] objColor;
                    if (trackable instanceof Point) {
                        objColor = new float[]{66.0f, 133.0f, 244.0f, 255.0f};
                    } else if (trackable instanceof Plane) {
                        objColor = new float[]{139.0f, 195.0f, 74.0f, 255.0f};
                        System.out.print(7);
                    } else {
                        objColor = DEFAULT_COLOR;
                    }

                    // Adding an Anchor tells ARCore that it should track this position in
                    // space. This anchor is created on the Plane to place the 3D model
                    // in the correct position relative both to the world and to the plane.
                    anchors.add(new ColoredAnchor(hit.createAnchor(), objColor));
                    break;
                }
            }
        }
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
            if(width == -1)
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
            if(width == -1)
                continue;
            if (width < min_Z_width)
                min_Z_width = width;
        }
        return min_X_width < min_Z_width ? min_X_width : min_Z_width;
    }


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
     * @param: points all points recognized in the frame
     * @param: floor the floor plane
     * @return: ArrayList of the objects in the session
     */

    // projects a pose on to a given plane
    private Pose project_pose_to_plane(Plane plane, Pose pose) {
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
        target_translation[0] = xPose + vector_length * a;
        target_translation[1] = yPose + vector_length * b;
        target_translation[2] = zPose + vector_length * c;
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

    private Pose[] insidePlane(Plane plane, Pose[] object) {
        Pose[] posesIndside_temp = new Pose[object.length - 1];
        int inserted_counter = 0;
        for (int i = 0; i < object.length - 1; i++) {
            if (plane.isPoseInPolygon(object[i])) {
                posesIndside_temp[i] = object[inserted_counter];
                inserted_counter++;
            }
        }
        Pose[] posesInside = new Pose[inserted_counter];
        for (int i = 0; i < posesInside.length; i++) {
            posesInside[i] = posesIndside_temp[i];
        }
        return posesInside;
    }

    private float[] firstAndLastProjection(float[][] projectedPoints) {
        float[] newOrder = new float[projectedPoints.length];
        float min_distance = 1000000;
        float[] firstPoint = new float[2];
        float[] lastPoint = new float[2];
        float minDistancePointIndex = 0;
        for (int i = 0; i < projectedPoints.length; i++) {

        }


        return newOrder;
    }

    private float distanceTwoPoints(float[] point1, float[] point2) {
        return (float) (Math.sqrt(Math.pow((double) (point1[0] - point2[0]), 2) +
                Math.pow((double) (point1[1] - point2[1]), 2)));
    }

    private float findFinalDistance(float[] polygon, float[][] objectsInside, TwoDLine xAxis){
        float [] newPolygon = TwoDLine.convert_points_to_coord(polygon, xAxis);
        return findFinalDistance(newPolygon, objectsInside, 'X', true, xAxis);
    }


    private float findFinalDistance(float[] polygon, float[][] objectsInside, char axis, boolean meter, TwoDLine meterAxis)
    //each object is sorted as [minX, minZ, maxX, maxZ] in the float-arrays array
    {
        //parameters for the resolution
        double EDGE_FACTOR = 0.5; //which distance from edges we stop checking
        if (meter) {
            EDGE_FACTOR = 0;
            if (meterAxis.getA() == 0)
                axis = 'X';
            else if (-meterAxis.getB()/meterAxis.getA() > 1 || -meterAxis.getB()/meterAxis.getA() < -1)
                axis = 'X';
            else
                axis = 'Z';
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

        float minimalMaxDistance = 999999999;

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
        float[] new_plane = change_plane(plane, edge_points, pointsIn);
        return new_plane;
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

    private float[] change_plane(float[] plane, float[] sides, float[] new_points) {
        for (int i = 0; i < plane.length - 1; i++) {
            if (plane[i] == sides[0] && plane[i + 1] == sides[1]) {
                boolean done = false;
                for (int j = 0; j < plane.length && !done; j++) {
                    if (plane[j] == sides[6] && plane[j + 1] == sides[7]) {
                        done = true;
                        if (i < j) {
                            float[] new_plane = new float[plane.length + i / 2 - j / 2 + 2];
                            for (int g = 0; g < i + 2; g++) {
                                new_plane[g] = plane[g];
                            }
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

    private float[][] sort_plane(float[] plane) {
        float[][] sorted = new float[2][plane.length / 2];
        float[] planesX = new float[plane.length / 2];
        float[] planesZ = new float[plane.length / 2];
        for (int i = 0; i < plane.length; i++) {
            sorted[1][i] = plane[2 * i];
            sorted[2][i] = plane[2 * i + 1];
        }
        return sorted;
    }


    /*private float[] projected_plane (float[] polygon, float[][] projected_points)
    {
        float[] polygon_x = new float [polygon.length/2];
        float[] polygon_z = new float [polygon.length/2];
        for (int i = 0; i < polygon.length/2; i++)
        {
            polygon_x[i] = polygon[2*i];
            polygon_z[i] = polygon[2*i + 1];
        }

        for (int i=0; i<projected_points.length;i++) {
            float[] projected_x = new float[projected_points.length / 2];
            float[] projected_z = new float[projected_points.length / 2];
            for (int j = 0; j < projected_points.length / 2; j++)
            {
                polygon_x[i] = projected_points[i][2 * i];
                polygon_z[i] = projected_points[i][2 * i + 1];
            }

        }

        return polygon;
    }
    */
    public boolean is_in_polygon(Plane plane, Pose[] object) {
        for (int i = 0; i < object.length; i++) {
            if (!plane.isPoseInPolygon(object[i]))
                return false;
        }
        return true;
    }

    //public float[] sort_figure(float[][] figure)
        /* gets an array of points on the plane (2 cell float array) - the first and the last
        points are those which intersect the polygon*/


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

    static float[] bufferToArray(FloatBuffer f){
        float[] points = new float[f.remaining()];
        for(int i = 0; i<points.length;i++)
        {
            points[i] = f.get();
        }
        f.rewind();
        return points;
    }

    /**
     * @param point     point to add to allPoints
     * @param allPoints all the points added so far
     * @return the index to which we need to add the new point
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
                if(max - min == 1)
                {
                    return max;
                }
                min = (max + min) / 2;
            }
        }
        return min;
    }


    /**
     * @param point the point to search it's object
     * @return index of object in objects in which the anchor is placed
     */
    public static float[] Convert_Point_From_Reality_to_Plane_Given_Angle(Pose point, Plane plane) {
/*
Main!!!
 */     float[] XAxis = plane.getCenterPose().getXAxis();
        float[] ZAxis = plane.getCenterPose().getZAxis();
        float xp = point.tx() * XAxis[0] + point.tz() * XAxis[2];
        float zp = point.tx() * ZAxis[0] + point.tz() * ZAxis[2];
        return new float[]{xp, zp};
    }


    private float convert_bool_to_float(boolean bool) {
        if (bool)
            return 1;
        else
            return 0;
    }

    private float[][] four_points_of_all_objects(Plane plane, ArrayList<ArrayList<float[]>> all_object_points) {
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

    private ArrayList<float[]> convertObjectToPlane(Plane plane, ArrayList<float[]> object_points, PointCloud cloud)
    {
        ArrayList<float[]> objectOnPlane = new ArrayList<>(object_points.size()) ;
        int count = 0;
        for(float[] p : object_points)
        {
            Pose pose = new Pose(p,new float[]{0,0,0,0});
            float[] point_in_plane_system = Convert_Point_From_Reality_to_Plane_Given_Angle(pose, plane);
            objectOnPlane.add(point_in_plane_system);
        }
        return objectOnPlane;
    }

    private ArrayList<float[]> convertObjectsToPlane(Plane plane, ArrayList<ArrayList<float[]>> objects, PointCloud cloud)
    {
        ArrayList<float[]> objectsOnPlane = new ArrayList<>();
        for (ArrayList<float[]> object : objects)
        {
            ArrayList<float[]> oneObject = convertObjectToPlane(plane, object, cloud);
            float[] thisObject = new float[2*oneObject.size()];
            for(int i = 0; i < thisObject.length; i+=2)
            {
                thisObject[i] = oneObject.get(i/2)[0];
                thisObject[i+1] = oneObject.get(i/2)[1];
            }
            if (thisObject.length > 6)
                objectsOnPlane.add(thisObject);
        }
        return objectsOnPlane;
    }


    private float[] four_points_of_object(Plane plane, ArrayList<float[]> object_points)
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
                    } else if (max(floor.getExtentZ(),floor.getExtentX()) < max(plane.getExtentX(), plane.getExtentZ())) {
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



    public static float Find_Rotation_Between_Coordinates(Plane p, PointCloud cloud) {
        return TwoDLine.Get_Rotation_Angle(p);
    }

    public static float[] Convert_Point_From_Reality_to_Plane(Plane plane, PointCloud cloud, Pose point) {
        /*
        Main!!!
         */
        float angle = Find_Rotation_Between_Coordinates(plane, cloud);
        float xp = (float) (-point.tx() * Math.cos(angle) - point.tz() * Math.sin(angle));
        float zp = (float) (-point.tx() * Math.sin(angle) - point.tz() * (Math.cos(angle)));
        return new float[]{xp, zp};
    }


    private float[] createPlane (PointCloud cloud, Plane plane, float[] polygon, ArrayList<ArrayList<float[]>> objects)
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


    private float merge_plane_width (PointCloud cloud, Plane plane, float[] polygon, ArrayList<ArrayList<float[]>> objects) {
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

    public float meter_ahead (Plane plane, float[] polygon, float[][] objectsInside, PointCloud pointCloud, Camera camera){
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
        System.out.print("works");
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

            if (diagonaCamera.Distance_To_Point_Not_Abs(polygon[i], polygon[i+1])/
                    diagonaCamera.Distance_To_Point_Not_Abs(polygon[i+2],polygon[i+3]) < 0){
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
        for (int i = 0; i < polygon.length ; i+= 2){
            if (i == cutInexes[0]){
                for (int j = i; j < polygon.length * 2; j += 2){
                    j = j & polygon.length;
                    if(j == cutInexes[1]) {
                        newPolygon[newPolygonCounter] = cameraPoseOnPlane[0];
                        newPolygon[newPolygonCounter] = cameraPoseOnPlane[1];
                        newPolygonCounter += 2;
                        int temp = polygon.length;
                        for (int k = j; k < polygon.length * 2; k++) {
                            if (k == cutInexes[2] || k == cutInexes[3]) {
                                while (k != cutInexes[3] && k != cutInexes[2]) {
                                    newPolygon[newPolygonCounter] = polygon[k];
                                    newPolygonCounter += 1;
                                }
                                newPolygon[newPolygonCounter] = meterAheadOnPlane[0];
                                newPolygon[newPolygonCounter] = meterAheadOnPlane[1];
                                newPolygonCounter += 2;
                                temp = k++;
                                break;
                            }
                            temp = k++;
                        }
                        for (int f = temp; f < polygon.length * 2; f++) {
                            if (f == cutInexes[2] || f == cutInexes[3])
                                while (f != cutInexes[0]) {
                                    newPolygon[newPolygonCounter] = polygon[f];
                                    newPolygonCounter++;
                                    f++;
                                }
                        }
                        break;
                    }
                    else
                    if(j == cutInexes[2] || j == cutInexes[3]){
                        for(int k = i; k < j + 2; k++){
                            newPolygon[newPolygonCounter] = polygon[k];
                            newPolygonCounter += 1;
                        }
                        newPolygon[newPolygonCounter] = meterAheadOnPlane[0];
                        newPolygon[newPolygonCounter] = meterAheadOnPlane[1];
                        newPolygonCounter += 2;
                        for (int k = j+1 ; k < polygon.length * 2; k++){
                            if (k == cutInexes[2] || k == cutInexes[3]){
                                while(k != cutInexes[1]){
                                    newPolygon[newPolygonCounter] = polygon[k];
                                    newPolygonCounter ++;
                                    k++;
                                }
                                break;
                            }
                        }
                        break;
                    }
                }
            }
        }
        return findFinalDistance(polygon, objectsInside, diagonaCamera);
    }
}

