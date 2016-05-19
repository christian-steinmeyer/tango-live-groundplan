/*
 * Copyright 2016 Google Inc. All Rights Reserved.
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

package com.favendo.steinmeyer.christian.tango.groundplan;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.os.AsyncTask;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.Toast;

import org.rajawali3d.math.Matrix4;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.scene.ASceneFrameCallback;
import org.rajawali3d.surface.RajawaliSurfaceView;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import com.projecttango.examples.java.floorplan.R;
import com.projecttango.rajawali.DeviceExtrinsics;
import com.projecttango.rajawali.ScenePoseCalculator;
import com.projecttango.tangosupport.TangoPointCloudManager;
import com.projecttango.tangosupport.TangoSupport;
import com.projecttango.tangosupport.TangoSupport.IntersectionPointPlaneModelPair;

/**
 * An example showing how to build a very simple application that allows the user to create a floor
 * plan in Java. It uses TangoSupportLibrary to do plane fitting using the point cloud data. When
 * the user clicks on the display, plane detection is done on the surface at the location of the
 * click and a 3D object will be placed in the scene anchored at that location. A {@code
 * WallMeasurement} will be recorded for that plane.
 * <p/>
 * You need to take exactly one measurement per wall in clockwise order. As you take measurements,
 * the perimeter of the floor plan will be displayed as lines in AR. After you have taken all the
 * measurements you can press the 'Done' button and the final result will be drawn in 2D as seen
 * from above along with labels showing the sizes of the walls.
 * <p/>
 * You are going to be building an ADF as you take the measurements. After pressing the 'Done'
 * button the ADF will be saved and an optimization will be run on it. After that, all the recorded
 * measurements are re-queried and the floor plan will be rebuilt in order to have better
 * precision.
 * <p/>
 * Note that it is important to include the KEY_BOOLEAN_LOWLATENCYIMUINTEGRATION configuration
 * parameter in order to achieve the best results synchronizing the Rajawali virtual world with the
 * RGB camera.
 * <p/>
 * For more details on the augmented reality effects, including color camera texture rendering, see
 * java_augmented_reality_example or java_hello_video_example.
 */
public class FloorplanActivity extends Activity implements View.OnTouchListener {
    private static final String TAG = FloorplanActivity.class.getSimpleName();
    // Record Device to Area Description as the main frame pair to be used for device pose
    // queries.
    private static final TangoCoordinateFramePair FRAME_PAIR =
            new TangoCoordinateFramePair(TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION,
                    TangoPoseData.COORDINATE_FRAME_DEVICE);
    private static final int INVALID_TEXTURE_ID = 0;

    private RajawaliSurfaceView mSurfaceView;
    private FloorplanRenderer mRenderer;
    private TangoCameraIntrinsics mIntrinsics;
    private DeviceExtrinsics mExtrinsics;
    private TangoPointCloudManager mPointCloudManager;
    private Tango mTango;
    private boolean mIsConnected = false;
    private boolean mIsMeasuring = false;
    private double mCameraPoseTimestamp = 0;
    private List<WallMeasurement> mWallMeasurementList;
    private Floorplan mFloorplan;
    private FinishPlanTask mFinishPlanTask;
    private Button mDoneButton;
    private Button mMeasureButton;
    private ViewGroup mProgressGroup;

    // Texture rendering related fields
    // NOTE: Naming indicates which thread is in charge of updating this variable
    private int mConnectedTextureIdGlThread = INVALID_TEXTURE_ID;
    private AtomicBoolean mIsFrameAvailableTangoThread = new AtomicBoolean(false);
    private double mRgbTimestampGlThread;

    // Time
    private static final int SECS_TO_MILLISECS = 1000;
    private double mXyIjPreviousTimeStamp;
    private static final double UPDATE_INTERVAL_MS = 500.0;
    private double mXyzIjTimeToNextUpdate = UPDATE_INTERVAL_MS;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mSurfaceView = (RajawaliSurfaceView) findViewById(R.id.ar_view);
        mRenderer = new FloorplanRenderer(this);
        mSurfaceView.setSurfaceRenderer(mRenderer);
        mSurfaceView.setOnTouchListener(this);
        // Set ZOrderOnTop to false so the other views don't get hidden by the SurfaceView.
        mSurfaceView.setZOrderOnTop(false);
        mProgressGroup = (ViewGroup) findViewById(R.id.progress_group);
        mPointCloudManager = new TangoPointCloudManager();
        mWallMeasurementList = new ArrayList<WallMeasurement>();
        mDoneButton = (Button) findViewById(R.id.done_button);
        mDoneButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                finishPlan();
            }
        });
        mMeasureButton = (Button) findViewById(R.id.measure_button);
        mMeasureButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                mIsMeasuring = !mIsMeasuring;
            }
        });
    }

    @Override
    protected void onPause() {
        super.onPause();
        // Synchronize against disconnecting while the service is being used in the OpenGL thread or
        // in the UI thread.
        synchronized (this) {
            if (mIsConnected) {
                mRenderer.getCurrentScene().clearFrameCallbacks();
                mTango.disconnectCamera(TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
                // We need to invalidate the connected texture ID so that we cause a re-connection
                // in the OpenGL thread after resume
                mConnectedTextureIdGlThread = INVALID_TEXTURE_ID;
                mTango.disconnect();
                mIsConnected = false;
            }
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        // Check if it has permissions.
        // Area learning permissions are needed in order to save the adf.
        if (Tango.hasPermission(this, Tango.PERMISSIONTYPE_ADF_LOAD_SAVE)) {
            connectAndStart();
        } else {
            startActivityForResult(
                    Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_ADF_LOAD_SAVE),
                    Tango.TANGO_INTENT_ACTIVITYCODE);
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Check which request we're responding to
        if (requestCode == Tango.TANGO_INTENT_ACTIVITYCODE) {
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, "Area Learning Permissions Required!", Toast.LENGTH_SHORT)
                        .show();
                finish();
            }
        }
    }

    /**
     * Connect to Tango service and connect the camera to the renderer.
     */
    private void connectAndStart() {
        // Synchronize against disconnecting while the service is being used in the OpenGL thread or
        // in the UI thread.
        synchronized (this) {
            if (!mIsConnected) {
                // Initialize Tango Service as a normal Android Service, since we call
                // mTango.disconnect() in onPause, this will unbind Tango Service, so
                // everytime when onResume get called, we should create a new Tango object.
                mTango = new Tango(FloorplanActivity.this, new Runnable() {
                    // Pass in a Runnable to be called from UI thread when Tango is ready,
                    // this Runnable will be running on a new thread.
                    // When Tango is ready, we can call Tango functions safely here only
                    // when there is no UI thread changes involved.
                    @Override
                    public void run() {
                        try {
                            connectTango();
                            connectRenderer();
                            mIsConnected = true;
                        } catch (TangoOutOfDateException e) {
                            Log.e(TAG, getString(R.string.exception_out_of_date), e);
                        }
                    }
                });
            }
        }
    }

    /**
     * Configure the Tango service and connect it to callbacks.
     */
    private void connectTango() {
        // Use default configuration for Tango Service, plus low latency
        // IMU integration and area learning.
        TangoConfig config = mTango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        // NOTE: Low latency integration is necessary to achieve a precise alignment of virtual
        // objects with the RGB image and produce a good AR effect.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_LOWLATENCYIMUINTEGRATION, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        // NOTE: Area learning is necessary to achieve better precision is pose estimation
        config.putBoolean(TangoConfig.KEY_BOOLEAN_LEARNINGMODE, true);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA, true);
        mTango.connect(config);

        // No need to add any coordinate frame pairs since we are not
        // using pose data. So just initialize.
        ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
        mTango.connectListener(framePairs, new OnTangoUpdateListener() {
            @Override
            public void onPoseAvailable(TangoPoseData pose) {
                // We are not using OnPoseAvailable for this app.
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                // Check if the frame available is for the camera we want and update its frame
                // on the view.
                if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                    // Mark a camera frame is available for rendering in the OpenGL thread
                    mIsFrameAvailableTangoThread.set(true);
                    mSurfaceView.requestRender();
                }
            }

            @Override
            public void onXyzIjAvailable(TangoXyzIjData xyzIj) {
                // Save the cloud and point data for later use.
                mPointCloudManager.updateXyzIj(xyzIj);
                if (mIsMeasuring) {
                    measureForWalls();
                }
            }

            @Override
            public void onTangoEvent(TangoEvent event) {
                // We are not using OnTangoEvent for this app.
            }
        });

        // Get extrinsics from device for use in transforms. This needs
        // to be done after connecting Tango and listeners.
        mExtrinsics = setupExtrinsics(mTango);
        mIntrinsics = mTango.getCameraIntrinsics(TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
    }

    private void calculateMxyzijTimeToNextUpdate(TangoXyzIjData xyzIj) {
        final double currentTimeStamp = xyzIj.timestamp;
        final double pointCloudFrameDelta =
                (currentTimeStamp - mXyIjPreviousTimeStamp) * SECS_TO_MILLISECS;
        mXyIjPreviousTimeStamp = currentTimeStamp;

        mXyzIjTimeToNextUpdate -= pointCloudFrameDelta;
    }

    /**
     * Connects the view and renderer to the color camera and callbacks.
     */
    private void connectRenderer() {
        // Register a Rajawali Scene Frame Callback to update the scene camera pose whenever a new
        // RGB frame is rendered.
        // (@see https://github.com/Rajawali/Rajawali/wiki/Scene-Frame-Callbacks)
        mRenderer.getCurrentScene().registerFrameCallback(new ASceneFrameCallback() {
            @Override
            public void onPreFrame(long sceneTime, double deltaTime) {
                // NOTE: This is called from the OpenGL render thread, after all the renderer
                // onRender callbacks had a chance to run and before scene objects are rendered
                // into the scene.

                // Prevent concurrent access to {@code mIsFrameAvailableTangoThread} from the Tango
                // callback thread and service disconnection from an onPause event.
                synchronized (FloorplanActivity.this) {
                    // Don't execute any tango API actions if we're not connected to the service
                    if (!mIsConnected) {
                        return;
                    }

                    // Set-up scene camera projection to match RGB camera intrinsics
                    if (!mRenderer.isSceneCameraConfigured()) {
                        mRenderer.setProjectionMatrix(mIntrinsics);
                    }

                    // Connect the camera texture to the OpenGL Texture if necessary
                    // NOTE: When the OpenGL context is recycled, Rajawali may re-generate the
                    // texture with a different ID.
                    if (mConnectedTextureIdGlThread != mRenderer.getTextureId()) {
                        mTango.connectTextureId(TangoCameraIntrinsics.TANGO_CAMERA_COLOR,
                                mRenderer.getTextureId());
                        mConnectedTextureIdGlThread = mRenderer.getTextureId();
                        Log.d(TAG, "connected to texture id: " + mRenderer.getTextureId());
                    }

                    // If there is a new RGB camera frame available, update the texture with it
                    if (mIsFrameAvailableTangoThread.compareAndSet(true, false)) {
                        mRgbTimestampGlThread =
                                mTango.updateTexture(TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
                    }

                    // If a new RGB frame has been rendered, update the camera pose to match.
                    if (mRgbTimestampGlThread > mCameraPoseTimestamp) {
                        // Calculate the device pose at the camera frame update time.
                        TangoPoseData lastFramePose =
                                mTango.getPoseAtTime(mRgbTimestampGlThread, FRAME_PAIR);
                        if (lastFramePose.statusCode == TangoPoseData.POSE_VALID) {
                            // Update the camera pose from the renderer
                            mRenderer.updateRenderCameraPose(lastFramePose);
                            mCameraPoseTimestamp = lastFramePose.timestamp;
                        } else {
                            Log.w(TAG, "Can't get device pose at time: " + mRgbTimestampGlThread);
                        }
                    }
                }
            }

            @Override
            public void onPreDraw(long sceneTime, double deltaTime) {

            }

            @Override
            public void onPostFrame(long sceneTime, double deltaTime) {

            }

            @Override
            public boolean callPreFrame() {
                return true;
            }
        });
    }

    /**
     * Calculates and stores the fixed transformations between the device and the various sensors to
     * be used later for transformations between frames.
     */
    private static DeviceExtrinsics setupExtrinsics(Tango tango) {
        // Create camera to IMU transform.
        TangoCoordinateFramePair framePair = new TangoCoordinateFramePair();
        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR;
        TangoPoseData imuTrgbPose = tango.getPoseAtTime(0.0, framePair);

        // Create device to IMU transform.
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;
        TangoPoseData imuTdevicePose = tango.getPoseAtTime(0.0, framePair);

        // Create depth camera to IMU transform.
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH;
        TangoPoseData imuTdepthPose = tango.getPoseAtTime(0.0, framePair);

        return new DeviceExtrinsics(imuTdevicePose, imuTrgbPose, imuTdepthPose);
    }

    /**
     * This method handles when the user clicks the screen. It will try to fit a plane to the
     * clicked point using depth data. The floor plan will be rebuilt and the result will be shown
     * in AR.
     */
    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {
        if (motionEvent.getAction() == MotionEvent.ACTION_UP) {
//            measureForWalls();
        }
        return true;
    }

    private void measureForWalls() {
        // Synchronize against concurrent access to the RGB timestamp in the
        // OpenGL thread and a possible service disconnection due to an onPause
        // event.
        WallMeasurement wallMeasurement;
        synchronized (this) {
            wallMeasurement = doWallMeasurement(mRgbTimestampGlThread);
        }

        // If the measurement was successful add it and run the floor plan building
        // algorithm.
        if (wallMeasurement != null) {
            if (isNewWall(wallMeasurement)) {
                mWallMeasurementList.add(wallMeasurement);
                mRenderer.addWallMeasurement(wallMeasurement);
                buildPlan(false);
            }
        }
    }

    private boolean isNewWall(WallMeasurement newWall) {
        for (WallMeasurement wall : mWallMeasurementList) {
            double angle = getAngle(newWall.getPlanePose().rotation, wall.getPlanePose().rotation);
            if (angle < 0.05) {
                return false;
            }
        }
        return true;
    }

    private double getAngle(double[] orientation, double[] other) {

        Quaternion wallOrientation =
                new Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]);
        Quaternion newWallOrientation = new Quaternion(other[0], other[1], other[2], other[3]);
        wallOrientation.normalize();
        newWallOrientation.normalize();
        return wallOrientation.angleBetween(newWallOrientation);
    }

    /**
     * Calculates the angle between two planes according to http://mathworld.wolfram
     * .com/DihedralAngle.html
     */
    private double getAngleBetweenPlanes(double[] a, double[] b) {
        double numerator = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
        double aFactor = Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        double bFactor = Math.sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
        double denumerator = aFactor * bFactor;
        double result = Math.acos(numerator / denumerator);
        return result;
    }

    /**
     * Use the TangoSupport library and point cloud data to calculate the plane at the specified
     * location in the color camera frame. It returns the pose of the fitted plane in a
     * TangoPoseData structure.
     */
    private WallMeasurement doWallMeasurement(double rgbTimestamp) {
        TangoXyzIjData xyzIj = mPointCloudManager.getLatestXyzIj();

        if (xyzIj == null) {
            return null;
        }

        // We need to calculate the transform between the color camera at the
        // time the user clicked and the depth camera at the time the depth
        // cloud was acquired.
        TangoPoseData colorTdepthPose = TangoSupport
                .calculateRelativePose(rgbTimestamp, TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR,
                        xyzIj.timestamp, TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH);

        // Get the device pose at the time the plane data was acquired.
        TangoPoseData devicePose = mTango.getPoseAtTime(xyzIj.timestamp, FRAME_PAIR);

        IntersectionPointPlaneModelPair wall =
                measureWall(xyzIj, devicePose, colorTdepthPose, 4, 4, 10);
        if (wall == null) {
            return null;
        }


        // Update the AR object location.
        TangoPoseData planeFitPose =
                calculatePlanePose(wall.intersectionPoint, wall.planeModel, devicePose);

        return new WallMeasurement(planeFitPose, devicePose);
    }

    /**
     * This method returns an {@link IntersectionPointPlaneModelPair} that is based on the
     * originally touched point, but also takes its environment into account. It will additionally
     * consider <code>accuracy</code> number of points and tries to fit an average plane for which
     * only the ones closer than 5% to the original one are considered.
     *
     * @param xyzIj
     *         required to create planes
     * @param devicePose
     *         required to create planes
     * @param colorTdepthPose
     *         required to create planes
     * @param horizontals
     *         number of horizontal measure points (in one line)
     * @param verticals
     *         number of vertical measure points (in one column)
     * @param minCommons
     *         number of common measure points required for positive feedback     @return average of
     *         all input information
     */
    private IntersectionPointPlaneModelPair measureWall(TangoXyzIjData xyzIj,
                                                        TangoPoseData devicePose,
                                                        TangoPoseData colorTdepthPose,
                                                        int horizontals, int verticals,
                                                        int minCommons) {

        List<IntersectionPointPlaneModelPair> candidates =
                getCandidates(xyzIj, devicePose, colorTdepthPose, horizontals, verticals);

        List<IntersectionPointPlaneModelPair> winners = removeOutliers(candidates);
        Log.i(TAG, candidates.size() + " candidates and " + winners.size() + " winners");
        if (winners.size() < minCommons) {
            return null;
        }

        return getAverage(winners);
    }

    private List<IntersectionPointPlaneModelPair> getCandidates(TangoXyzIjData xyzIj,
                                                                TangoPoseData devicePose,
                                                                TangoPoseData colorTdepthPose,
                                                                int horizontals, int verticals) {
        List<IntersectionPointPlaneModelPair> candidates = new ArrayList<>();

        float hStep = 1f / (horizontals + 1f);
        float vStep = 1f / (verticals + 1f);

        for (float x = hStep; x < 1f; x += hStep) {
            for (float y = vStep; y < 1f; y += vStep) {
                try {
                    IntersectionPointPlaneModelPair candidate = TangoSupport
                            .fitPlaneModelNearClick(xyzIj, mIntrinsics, colorTdepthPose, x, y);
                    if (isAlignedWithGravity(candidate, devicePose)) {
                        candidates.add(candidate);
                    }
                } catch (TangoException t) {
                    Log.e(TAG, getString(R.string.failed_measurement));
                } catch (SecurityException t) {
                    Log.e(TAG, getString(R.string.failed_permissions), t);
                }
            }
        }
        return candidates;
    }

    private boolean isAlignedWithGravity(IntersectionPointPlaneModelPair candidate,
                                         TangoPoseData devicePose) {
        Matrix4 adfTdevice = ScenePoseCalculator.tangoPoseToMatrix(devicePose);
        Vector3 gravityVector = ScenePoseCalculator.TANGO_WORLD_UP.clone();
        adfTdevice.clone().multiply(mExtrinsics.getDeviceTDepthCamera()).inverse().
                rotateVector(gravityVector);

        double[] gravity = new double[]{gravityVector.x, gravityVector.y, gravityVector.z};
        double angle = getAngleBetweenPlanes(candidate.planeModel, gravity);
        Log.d(TAG, "angle: " + angle);
        if (angle < 0.1) {
            return false;
        }
        return true;
    }

    private List<IntersectionPointPlaneModelPair> removeOutliers(
            List<IntersectionPointPlaneModelPair> candidates) {
        List<IntersectionPointPlaneModelPair> result = new ArrayList<>();
        boolean[][] candidatePairs = new boolean[candidates.size()][candidates.size()];

        for (int i = 0; i < candidates.size(); i++) {
            for (int j = i + 1; j < candidates.size(); j++) {
                double angle = getAngleBetweenPlanes(candidates.get(i).planeModel,
                        candidates.get(j).planeModel);
                Log.d(TAG, "Angle: " + angle);
                if (angle < 0.05) {
                    candidatePairs[i][j] = true;
                    candidatePairs[j][i] = true;
                }
            }
        }

        int max = 0;
        int index = -1;
        for (int i = 0; i < candidates.size(); i++) {
            int pairCount = 0;
            for (int j = 0; j < candidates.size(); j++) {
                if (candidatePairs[i][j]) {
                    pairCount++;
                }
            }
            index = max < pairCount ? i : index;
            max = Math.max(max, pairCount);
        }

        if (index > -1) {
            for (int i = 0; i < candidates.size(); i++) {
                if (candidatePairs[index][i]) {
                    result.add(candidates.get(i));
                }
            }
        }
        return result;
    }

    private IntersectionPointPlaneModelPair getAverage(
            List<IntersectionPointPlaneModelPair> intersectionPointPlaneModelPairs) {

        double[] intersectionPoint = new double[3];
        double[] planeModel = new double[4];
        for (IntersectionPointPlaneModelPair each : intersectionPointPlaneModelPairs) {
            for (int i = 0; i < 3; i++) {
                intersectionPoint[i] += each.intersectionPoint[i];
                planeModel[i] += each.planeModel[i];
            }
            planeModel[3] += each.planeModel[3]; // not in own loop due to performance
        }
        for (int i = 0; i < 3; i++) {
            intersectionPoint[i] = intersectionPoint[i] / intersectionPointPlaneModelPairs.size();
            planeModel[i] = planeModel[i] / intersectionPointPlaneModelPairs.size();
        }
        planeModel[3] = planeModel[3] / intersectionPointPlaneModelPairs.size(); // see above
        return new TangoSupport.IntersectionPointPlaneModelPair(intersectionPoint, planeModel);
    }

    /**
     * Calculate the pose of the plane based on the position and normal orientation of the plane and
     * align it with gravity.
     */
    private TangoPoseData calculatePlanePose(double[] point, double normal[],
                                             TangoPoseData devicePose) {
        Matrix4 adfTdevice = ScenePoseCalculator.tangoPoseToMatrix(devicePose);
        // Vector aligned to gravity.
        Vector3 depthUp = ScenePoseCalculator.TANGO_WORLD_UP.clone();
        adfTdevice.clone().multiply(mExtrinsics.getDeviceTDepthCamera()).inverse().
                rotateVector(depthUp);
        // Create the plane matrix transform in depth frame from a point, the plane normal and the
        // up vector.
        Matrix4 depthTplane = ScenePoseCalculator.matrixFromPointNormalUp(point, normal, depthUp);
        // Plane matrix transform in adf frame.
        Matrix4 adfTplane =
                adfTdevice.multiply(mExtrinsics.getDeviceTDepthCamera()).multiply(depthTplane);

        TangoPoseData planeFitPose = ScenePoseCalculator.matrixToTangoPose(adfTplane);
        planeFitPose.baseFrame = TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION;
        // NOTE: We need to set the target frame to COORDINATE_FRAME_DEVICE because that is the
        // default target frame to place objects in the OpenGL world with
        // TangoSupport.getPoseInEngineFrame.
        planeFitPose.targetFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;

        return planeFitPose;
    }

    /**
     * Builds the plan from the set of measurements and updates the rendering in AR.
     *
     * @param closed
     *         If true, close the floor plan; if false, continue the floor plan.
     */
    public void buildPlan(boolean closed) {
        mFloorplan = PlanBuilder.buildPlan(mWallMeasurementList, closed);
        mRenderer.updatePlan(mFloorplan);
    }

    /**
     * Updates every saved measurement. It re-queries the device pose at the time the measurement
     * was taken.
     */
    public void updateMeasurements() {
        for (WallMeasurement wallMeasurement : mWallMeasurementList) {
            // We need to re query the device pose when the measurements were taken.
            TangoPoseData newDevicePose =
                    mTango.getPoseAtTime(wallMeasurement.getDevicePoseTimeStamp(), FRAME_PAIR);
            wallMeasurement.update(newDevicePose);
            mRenderer.addWallMeasurement(wallMeasurement);
        }
    }

    /**
     * Finish plan, save the adf, and show the final result. Executed as an AsyncTask because saving
     * the adf could be an expensive operation.
     */
    private void finishPlan() {
        if (mFinishPlanTask != null) {
            Log.w(TAG, "Finish task already executing");
            return;
        }

        mFinishPlanTask = new FinishPlanTask();
        mFinishPlanTask.execute();
    }

    /**
     * Finish plan AsyncTask. Shows a spinner while it's saving the adf and updating the
     * measurements. Draws the final result on a canvas and shows it.
     */
    private class FinishPlanTask extends AsyncTask<Void, Integer, Void> {

        @Override
        protected void onPreExecute() {
            mProgressGroup.setVisibility(View.VISIBLE);
        }

        @Override
        protected Void doInBackground(Void... params) {
            // Save and optimize ADF.
            mTango.saveAreaDescription();
            // Update poses after optimization and re build plan.
            mRenderer.removeMeasurements();
            updateMeasurements();
            buildPlan(true);

            return null;
        }

        @Override
        protected void onPostExecute(Void v) {
            mProgressGroup.setVisibility(View.GONE);
            // Draw final result on Canvas.
            PlanView planView = new PlanView(FloorplanActivity.this);
            planView.setLayoutParams(
                    new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,
                            LinearLayout.LayoutParams.MATCH_PARENT));
            setContentView(planView);
            planView.invalidate();
            mFinishPlanTask = null;
        }
    }

    /**
     * Custom View that draws the plan in 2D.
     */
    private class PlanView extends View {

        private Paint mPaint;

        public PlanView(Context context) {
            super(context);
            mPaint = new Paint();
            mPaint.setStrokeWidth(10);
            mPaint.setTextSize(50);
        }

        @Override
        public void onDraw(Canvas canvas) {
            mFloorplan.drawOnCanvas(canvas, mPaint);
        }
    }
}
