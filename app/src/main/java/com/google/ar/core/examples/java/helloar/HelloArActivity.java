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
import java.util.List;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

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

                        float w = TwoDLine.find_width(((Plane) trackable).getPolygon().array());
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
     * computes the distance between to poses
     *
     * @param pose1 first pose
     * @param pose2 second pose
     * @return the distance between the poses
     */
    private float distanceBetweenPoses(Pose pose1, Pose pose2) {
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
    private float distanceBetweenPoses(float[] pose1, float[] pose2) {
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

    private float findFinalDistance(float[] polygon, float[][] objectsInside, char axis)
    //each object is sorted as [minX, minZ, maxX, maxZ] in the float-arrays array
    {
        //parameters for the resolution
        double EDGE_FACTOR = 0.3; //which distance from edges we stop checking
        double STEP_FACTOR = 0.1; //the stepsize we go in each axis
        int MAX_POINTS = 10; // the maximum amount of objects we estimate to be in one axis
        float maxX = 0, minX = 0, maxZ = 0, minZ = 0;
        //find the minimal and maximal x and z coordinates in the polygon
        for (int i = 0; i < polygon.length; i++) {
            if (polygon[2 * i] < minX)
                minX = polygon[2 * i];
            else if (polygon[2 * i] > maxX)
                maxX = polygon[2 * i];
            if (polygon[2 * i + 1] < minZ)
                minZ = polygon[2 * i + 1];
            else if (polygon[2 * i + 1] > maxZ)
                maxZ = polygon[2 * i + 1];
        }

        float minimalMaxDistance = 0;

        for (double i = minX + EDGE_FACTOR; i < maxX - EDGE_FACTOR; i += STEP_FACTOR) {
            float current_maximum = 0;
            float a, b, c;
            if (axis == 'x') {
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
            float[] onDiameter = new float[MAX_POINTS * 2];//the array in which we put the points which are on our Axis
            int added_counter = 0; // counts how many coordinates we added (pointsX2)
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

                float max_distance = 0;
                for (int k = 0; k < onDiameter.length && onDiameter[k] != 0.0f; k++)
                    // check the maximum passable length between objects
                    if (TwoDLine.Distance_Between_Points(onDiameter[4 * k], onDiameter[4 * k + 1],
                            onDiameter[4 * k + 2], onDiameter[4 * k + 3]) > max_distance)
                        max_distance = TwoDLine.Distance_Between_Points(onDiameter[4 * k],
                                onDiameter[4 * k + 1], onDiameter[4 * k + 2], onDiameter[4 * k + 3]);

                if (max_distance < minimalMaxDistance) // we need to find the minimal maximal size to know in which distance we can pass
                    minimalMaxDistance = max_distance;
            }
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


    private ArrayList<ArrayList<float[]>> getObjects(FloatBuffer points, Plane floor) {
        //Collection<Anchor> myAnchors = session.getAllAnchors();
        ArrayList<float[]> allPoints = new ArrayList<>();
        for (int i = 0; i < points.remaining(); i += 4) {
            float[] currPoint = {points.get(i), points.get(i + 1), points.get(i + 2), points.get(i + 3)};
            Pose pointPose = new Pose(currPoint, new float[]{0.0f, 0.0f, 0.0f, 0.0f});
            // make sure the point isn't on the floor and that we are sure enough about it's position
            if (currPoint[3] > 0.4) {“ //&& !floor.isPoseInExtents(pointPose)) {
                allPoints.add(binarySearch(currPoint, allPoints), currPoint);
            }
        }
        ArrayList<ArrayList<float[]>> objects = new ArrayList<>();
        float distanceThreshold = 0.1f;
        for (float[] currPoint : allPoints) {
            // check that the point isn't on the floor
            // threshold of security about point position
            boolean foundObject = false;
            for (ArrayList<float[]> object : objects) {
                if (foundObject) {
                    break;
                }
                int index = binarySearch(currPoint, object);
                if (distanceBetweenPoses(currPoint, allPoints.get(index)) < distanceThreshold) {
                    object.add(index, currPoint);
                    foundObject = true;
                }
                if (index != 0 && distanceBetweenPoses(currPoint, allPoints.get(index - 1)) < distanceThreshold) {
                    object.add(index, currPoint);
                    foundObject = true;
                }
                if (index != allPoints.size() - 1 && distanceBetweenPoses(currPoint, allPoints.get(index + 1)) < distanceThreshold) {
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
        return objects;
    }


    /**
     * @param point     point to add to allPoints
     * @param allPoints all the points added so far
     * @return the index to which we need to add the new point
     */
    int binarySearch(float[] point, ArrayList<float[]> allPoints) {
        if (allPoints.size() == 0) {
            return -1;
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
                min = (max + min) / 2;
            }
        }
        return -1;
    }

    /**
     * @param point   the point to search it's object
     * @param objects all objects found
     * @return index of object in objects in which the anchor is placed
     */
    private int getIndexOfObject(float[] point, ArrayList<ArrayList<float[]>> objects) {
        for (ArrayList<float[]> object : objects) {
            boolean contains = false;
            for (float[] pose1 : object) {
                float pose2[] = point;
                if (pose1[0] == pose2[0] && pose1[1] == pose2[1] && pose1[2] == pose2[2]) {
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
    private float pixelsToDistance(float[] pixel1, float[] pixel2, Frame frame) {
        List<HitResult> hits1 = frame.hitTest(pixel1[0], pixel1[1]);
        List<HitResult> hits2 = frame.hitTest(pixel2[0], pixel2[1]);
        if (hits1.isEmpty() || hits2.isEmpty()) {
            return -1.0f;
        }
        // set the minimal distance to be too large
        float minDistance = 300.0f;
        for (HitResult hit1 : hits1) {
            for (HitResult hit2 : hits2) {
                float distance = distanceBetweenPoses(hit1.getHitPose(), hit2.getHitPose());
                minDistance = (distance < minDistance) ? distance : minDistance;
            }
        }
        return minDistance;
    }

}

    public static float Find_Rotation_Between_Coordinates(Plane p, PointCloud cloud) {
        float[] points = TwoDLine.filter_plane_points(cloud, p.getCenterPose().ty());
        float x_center_pose = p.getCenterPose().tx();
        float z_center_pose = p.getCenterPose().tz();
        float[] point = TwoDLine.find_point_with_max_distance(x_center_pose, z_center_pose, points);
        return TwoDLine.Get_Rotation_Angle(x_center_pose, z_center_pose, point[0], point[1]);
    }

    public static float[] Convert_Point_From_Reality_to_Plane(Plane plane, PointCloud cloud, Pose point) {
        /*
        Main!!!
         */
        float angle = Find_Rotation_Between_Coordinates(plane, cloud);
        float xp = (float) (point.tx() * Math.cos(angle) - point.tz() * Math.sin(angle));
        float zp = (float) (point.tx() * Math.sin(angle) + point.tz() * (Math.cos(angle)));
        return new float[]{xp, zp};
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
                float[] real_object = four_points_of_object(plane, object, cloud);
                float[][] sorted = sort_points(polygon, real_object);
                float[] pointsIn = sorted[0];
                float[] pointsOut = sorted[1];
                polygon = cut_plane_points_inside(polygon, pointsIn, pointsOut);
            }
        }
        float Xdistance = findFinalDistance(polygon,objectsInside,'x');
        float Zdistance = findFinalDistance(polygon, objectsInside, 'z');
        return Math.min(Xdistance, Zdistance);
    }

}

