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

import android.media.Image;
import android.opengl.GLES20;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.MotionEvent;
import android.view.Surface;
import android.widget.TextView;
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

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.core.Mat;

import java.io.IOException;

import java.nio.FloatBuffer;
import java.util.ArrayList;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import static com.google.ar.core.examples.java.helloar.OCD.transformRatioToScreen;

/**
 * This is a simple example that shows how to create an augmented reality (AR) application using the
 * ARCore API. The application will display any detected planes and will allow the user to tap on a
 * plane to place a 3d model of the Android robot.
 */
public class HelloArActivity extends AppCompatActivity implements GLSurfaceView.Renderer {
    static {
        if (!OpenCVLoader.initDebug()) {
            // problem
        }
    }

    private static final String TAG = HelloArActivity.class.getSimpleName();

    // Rendering. The Renderers are created here, and initialized when the GL surface is created.
    private GLSurfaceView surfaceView;
    private DetectionView ourView;
    private CvHelper cvView;

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
    private OCD ocd;
    private long counter = 0;
    private int door_counter = 0;
    private float[] door_widths;
    private final int avg_times = 5;

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
    private boolean createdOCD = false;
    private TextView textView;
    private int screenHeight;
    private int screenWidth;
    private int realWidth;

    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    Log.i(TAG, "i don't know succes opencv something");
                    Mat imageMat = new Mat();
                }
                break;
                default: {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        OpenCVLoader.initDebug();
        super.onCreate(savedInstanceState);
        DisplayMetrics displayMetrics = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(displayMetrics);
        screenHeight = displayMetrics.heightPixels;
        screenWidth = displayMetrics.widthPixels;
        setContentView(R.layout.activity_main);
        surfaceView = findViewById(R.id.surfaceview);
        ourView = findViewById(R.id.ourView);
        cvView = findViewById(R.id.CVview);
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

        textView = findViewById(R.id.textView);

        installRequested = false;

        door_widths = new float[avg_times];
    }

    @Override
    protected void onResume() {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, this, mLoaderCallback);
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

//        messageSnackbarHelper.showMessage(this, "Searching for surfaces...");
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
            counter++;
            String message = " ";
            //#############################################


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
            pointCloudRenderer.update(pointCloud);
            pointCloudRenderer.draw(viewmtx, projmtx);
            if (counter % 20 == 0) {
                Image image = null;
                try {
                    try {
                        image = frame.acquireCameraImage();
                    } catch (Exception e) {
                        int a = 1;
                    }
                    // OCD CODE
                    if (!createdOCD) {
                        try {
                            ocd = OCD.create(getAssets(),
                                    "model.tflite",
                                    "labels.txt",
                                    300,
                                    false, image.getWidth(),
                                    image.getHeight(),
                                    displayRotationHelper.getRotation() - getScreenOrientation());
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                        createdOCD = true;
                    }

                    // important
                    int width = image.getHeight();
                    int height = image.getWidth();
                    realWidth = screenHeight * width / height;
                    ArrayList<OCD.Recognition> recognitions = ocd.detect(image);


                    //#############################################


                    //#############################################
                    // AR CODE

                    ArrayList<Plane> ALPlanes = new ArrayList<>(session.getAllTrackables(Plane.class));
                    floor = AR.getFloor(ALPlanes, camera);
                    // important
                    float floorWidth = -1.0f;
                    if (floor != null) {
                        FloatBuffer ptsBuffer = pointCloud.getPoints();
                        float[] allFramePoints = AR.bufferToArray(ptsBuffer);

                        ArrayList<float[]> ptsLst3D = AR.findExtensionPoints(floor, allFramePoints);
                        ArrayList<float[]> ptsLst2D = new ArrayList<>();
                        for (float[] point3D : ptsLst3D) {
                            Pose pTemp = new Pose(point3D, new float[4]);
                            pTemp = AR.project_pose_to_plane(floor, pTemp);
                            ptsLst2D.add(AR.Convert_Point_From_Reality_to_Plane_Given_Angle(pTemp, floor));
                        }
                        float[] floor2D = floor.getPolygon().array();
                        float[] floor2 = AR.extendFloor(floor2D, ptsLst2D, floor.getExtentX(), floor.getExtentZ());

                        floorWidth = AR.find_width(floor); // update to new find_width with cut objects
                        if (floorWidth < 10) {
                            message += "Floor width ";
                            message += floorWidth;
                            message += '\n';
                        }
                    }


                    // important
                    float[] object_widths = new float[recognitions.size()];
                    float[] center_of_objects = new float[recognitions.size()];

                    for (int i = 0; i < recognitions.size(); i++) {
                        // transform from 0-1 to screen height and width
                        float top = recognitions.get(i).location.top;
                        float bottom = recognitions.get(i).location.bottom;
                        float left = recognitions.get(i).location.left;
                        float right = recognitions.get(i).location.right;
                        float[] transformed_pixels = OCD.transformRatioToScreen(top, bottom, left, right, realWidth, screenWidth, screenHeight);
                        float[] pixel1 = {transformed_pixels[2], transformed_pixels[1]};
                        float[] pixel2 = {transformed_pixels[3], transformed_pixels[1]};
                        float[] leftUp = {transformed_pixels[2], transformed_pixels[0]};
                        float[] rightDown = {transformed_pixels[3], transformed_pixels[1]};
                        float[] leftDown = {transformed_pixels[2], transformed_pixels[1]};
                        float[] rightUp = {transformed_pixels[3], transformed_pixels[0]};
                        ourView.setRect(transformed_pixels[2], transformed_pixels[0], transformed_pixels[3], transformed_pixels[1]);
                        ourView.invalidate();
                        float[] centerPixel = {(transformed_pixels[2] + transformed_pixels[3]) / 2.0f, (transformed_pixels[0] + transformed_pixels[1]) / 2.0f};
                        center_of_objects[i] = AR.pixelToDistance(centerPixel, frame);
                        // transform from 300x300 to 640x480
                        float cv_top = recognitions.get(i).location.top * (float) height * 0.95f;
                        float cv_bottom = recognitions.get(i).location.bottom * (float) height * 1.05f;
                        cv_bottom = cv_bottom > height - 1 ? height - 1 : cv_bottom;
                        cv_top = cv_top < 0 ? 0 : cv_top;
                        float cv_left = recognitions.get(i).location.left * (float) height * 0.95f;
                        float cv_right = recognitions.get(i).location.right * (float) height * 1.05f;
                        cv_right = cv_right > height - 1 ? height - 1 : cv_right;
                        cv_left = cv_left < 0 ? 0 : cv_left;
                        int orientation = getScreenOrientation();
                        double[][] lines = ocd.imageProcess(image, cv_top, cv_bottom, cv_left, cv_right, orientation);
                        double[] left_line = lines[0];
                        double[] right_line = lines[1];
                        if (left_line != null && right_line != null) {
                            left_line[0] += cv_left;
                            left_line[2] += cv_left;
                            left_line[1] += cv_top;
                            left_line[3] += cv_top;
                            right_line[0] += cv_left;
                            right_line[2] += cv_left;
                            right_line[1] += cv_top;
                            right_line[3] += cv_top;
                            left_line[0] /= (double) width;
                            left_line[2] /= (double) width;
                            left_line[1] /= (double) height;
                            left_line[3] /= (double) height;
                            right_line[0] /= (double) width;
                            right_line[2] /= (double) width;
                            right_line[1] /= (double) height;
                            right_line[3] /= (double) height;
                            float[] transformed_cv_left = OCD.transformRatioToScreen((float) left_line[1], (float) left_line[3], (float) left_line[0], (float) left_line[2], realWidth, screenWidth, screenHeight);
                            float[] transformed_cv_right = OCD.transformRatioToScreen((float) right_line[1], (float) right_line[3], (float) right_line[0], (float) right_line[2], realWidth, screenWidth, screenHeight);
                            object_widths[i] = AR.findMinDistBetweenLines(new float[]{transformed_cv_left[2], transformed_cv_left[0]}, new float[]{transformed_cv_right[2], transformed_cv_right[0]}, new float[]{transformed_cv_left[3], transformed_cv_left[1]}, new float[]{transformed_cv_right[3], transformed_cv_right[1]}, frame);
                            Line leftDraw = new Line(transformed_cv_left[2], transformed_cv_left[0], transformed_cv_left[3], transformed_cv_left[1]);
                            Line rightDraw = new Line(transformed_cv_right[2], transformed_cv_right[0], transformed_cv_right[3], transformed_cv_right[1]);
                            cvView.cleanUp();
                            cvView.drawLine(leftDraw);
                            cvView.drawLine(rightDraw);
                            cvView.invalidate();
                        }

                        // somekind of show: width of lable is object_widths[i]
                        // somekind of show: distance of lable from phone is center_of_objects[i]
                    }
                    if (object_widths.length != 0 && object_widths[0] != -1 && recognitions.get(0).confidence > 0.7) {
                        message += "width of ";
                        message += recognitions.get(0).label;
                        message += " is ";
                        message += object_widths[0];
                        if (door_counter < avg_times) {
                            door_widths[door_counter] = object_widths[0];
                            door_counter++;
                        } else {
                            door_widths[door_counter % avg_times] = object_widths[0];
                            door_counter++;
                            float avg_width = 0.0f;
                            for (int k = 0; k < avg_times; k++) {
                                avg_width += door_widths[k];
                            }
                            avg_width /= avg_times;
                            message += "\n";
                            message += "AR-CV width is:";
                            message += avg_width;
                        }
                    } else if (object_widths.length != 0 && recognitions.get(0).confidence > 0.7) {
                        message += "recognized a ";
                        message += recognitions.get(0).label;
                    } else if (object_widths.length == 0) {
                        door_counter = 0;
                        cvView.cleanUp();
                    }
                    if (!message.equals("")) {
                        textView.setText(message);
                    }
                } catch (Throwable ignored) {
                    int a = 1;
                }
                if (image != null) {
                    image.close();
                }
            }

            // Application is responsible for releasing the point cloud resources after
            // using it.
            pointCloud.release();

            // Check if we detected at least one plane. If so, hide the loading message.
            if (messageSnackbarHelper.isShowing()) {
                for (Plane plane : session.getAllTrackables(Plane.class)) {
                    if (plane.getTrackingState() == TrackingState.TRACKING) {
                        //messageSnackbarHelper.hide(this);
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

    protected int getScreenOrientation() {
        switch (getWindowManager().getDefaultDisplay().getRotation()) {
            case Surface.ROTATION_270:
                return 270;
            case Surface.ROTATION_180:
                return 180;
            case Surface.ROTATION_90:
                return 90;
            default:
                return 0;
        }
    }
}