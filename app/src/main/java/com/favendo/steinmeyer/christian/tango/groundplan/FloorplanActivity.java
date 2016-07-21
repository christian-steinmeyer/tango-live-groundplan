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

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.graphics.Canvas;
import android.graphics.Color;
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
import com.projecttango.examples.java.floorplan.R;
import com.projecttango.rajawali.DeviceExtrinsics;
import com.projecttango.rajawali.ScenePoseCalculator;
import com.projecttango.tangosupport.TangoPointCloudManager;
import com.projecttango.tangosupport.TangoSupport;
import com.projecttango.tangosupport.TangoSupport.IntersectionPointPlaneModelPair;

import org.rajawali3d.math.Matrix4;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.scene.ASceneFrameCallback;
import org.rajawali3d.surface.RajawaliSurfaceView;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * An example showing how to build a very simple application that allows the user to create a floor
 * plan in Java. It uses TangoSupportLibrary to do plane fitting using the point cloud data. When
 * the user clicks on the display, plane detection is done on the surface at the location of the
 * click and a 3D object will be placed in the scene anchored at that location. A {@code
 * WallMeasurement} will be recorded for that plane.
 * <p>
 * You need to take exactly one measurement per wall in clockwise order. As you take measurements,
 * the perimeter of the floor plan will be displayed as lines in AR. After you have taken all the
 * measurements you can press the 'Done' button and the final result will be drawn in 2D as seen
 * from above along with labels showing the sizes of the walls.
 * <p>
 * You are going to be building an ADF as you take the measurements. After pressing the 'Done'
 * button the ADF will be saved and an optimization will be run on it. After that, all the recorded
 * measurements are re-queried and the floor plan will be rebuilt in order to have better
 * precision.
 * <p>
 * Note that it is important to include the KEY_BOOLEAN_LOWLATENCYIMUINTEGRATION configuration
 * parameter in order to achieve the best results synchronizing the Rajawali virtual world with the
 * RGB camera.
 * <p>
 * For more details on the augmented reality effects, including color camera texture rendering, see
 * java_augmented_reality_example or java_hello_video_example.
 */
public class FloorplanActivity extends Activity implements View.OnTouchListener {
    private static final String TAG = FloorplanActivity.class.getSimpleName();
    // Record Device to Area Description as the main frame pair to be used for device pose
    // queries.

    private StatusView status;
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
    private List<CornerMeasurement> mCornerMeasurementList;
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
    private static final double UPDATE_INTERVAL_MS = 1000.0 / 3;
    private double mXyzIjTimeToNextUpdate = UPDATE_INTERVAL_MS;

    // Accuracy
    private static final double MAX_DEVIATION = Math.PI / 90; // = 2°

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);

        status = new StatusView(this);
        getWindow().addContentView(status,
                new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,
                        ViewGroup.LayoutParams.MATCH_PARENT));

        mSurfaceView = (RajawaliSurfaceView) findViewById(R.id.ar_view);
        mRenderer = new FloorplanRenderer(this);
        mSurfaceView.setSurfaceRenderer(mRenderer);
        mSurfaceView.setOnTouchListener(this);
        // Set ZOrderOnTop to false so the other views don't get hidden by the SurfaceView.
        mSurfaceView.setZOrderOnTop(false);
        mProgressGroup = (ViewGroup) findViewById(R.id.progress_group);
        mPointCloudManager = new TangoPointCloudManager();
        mCornerMeasurementList = new ArrayList<>();
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
        ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<>();
        mTango.connectListener(framePairs, new OnTangoUpdateListener() {
            @Override
            public void onPoseAvailable(TangoPoseData pose) {
                if (pose.baseFrame == TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION &&
                        pose.targetFrame == TangoPoseData.COORDINATE_FRAME_DEVICE) {
                    // Process new ADF to device pose data.
                    Log.d(TAG, "NEW ADF TO DEVICE POSE DATA THAT NEEDS TO BE PROCESSED");
                } else if (pose.baseFrame == TangoPoseData.COORDINATE_FRAME_AREA_DESCRIPTION &&
                        pose.targetFrame == TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE) {
                    // Process new localization.
                    Log.d(TAG, "NEW LOCATION DATA THAT NEEDS TO BE PROCESSED");
                }
                updateMeasurements();
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
                if (mIsMeasuring && isMxyzijTimeToUpdate(xyzIj)) {
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

    private boolean isMxyzijTimeToUpdate(TangoXyzIjData xyzIj) {
        final double currentTimeStamp = xyzIj.timestamp;
        final double pointCloudFrameDelta =
                (currentTimeStamp - mXyIjPreviousTimeStamp) * SECS_TO_MILLISECS;
        mXyIjPreviousTimeStamp = currentTimeStamp;

        mXyzIjTimeToNextUpdate -= pointCloudFrameDelta;

        boolean result = mXyzIjTimeToNextUpdate < 0;
        if (result) {
            mXyzIjTimeToNextUpdate = UPDATE_INTERVAL_MS;
        }
        return result;
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
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    Toast.makeText(getBaseContext(), "No Pose available",
                                            Toast.LENGTH_SHORT);
                                }
                            });
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
     * This method handles when the user clicks the screen.
     */
    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {
        return true;
    }

    private void measureForWalls() {
        // Synchronize against concurrent access to the RGB timestamp in the
        // OpenGL thread and a possible service disconnection due to an onPause
        // event.
        CornerMeasurement cornerMeasurement;
        synchronized (this) {
            cornerMeasurement =
                    doCornerMeasurement(mPointCloudManager.getLatestXyzIj(), mExtrinsics,
                            mRgbTimestampGlThread);
        }

        // If the measurement was successful add it and run the floor plan building
        // algorithm.
        if (cornerMeasurement != null) {
            if (isNewCorner(cornerMeasurement)) {
                mCornerMeasurementList.add(cornerMeasurement);
                mRenderer.addCornerMeasurement(cornerMeasurement);
                buildPlan(false);
            }
        }
    }

    private boolean isNewCorner(CornerMeasurement newCorner) {
        for (CornerMeasurement corner : mCornerMeasurementList) {
            if (corner.equals(newCorner)) {
                return false;
            }
        }
        return true;
    }

    private double getAngleBetweenPlanes(double[] orientation, double[] other) {

        Quaternion wallOrientation =
                new Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]);
        Quaternion newWallOrientation = new Quaternion(other[0], other[1], other[2], other[3]);
        return wallOrientation.angleBetween(newWallOrientation);
    }


    /**
     * Use the TangoSupport library and point cloud data to calculate the plane at the specified
     * location in the color camera frame. It returns the pose of the fitted plane in a
     * TangoPoseData structure.
     */
    private CornerMeasurement doCornerMeasurement(TangoXyzIjData xyzIj, DeviceExtrinsics extrinsics,
                                                  double rgbTimestamp) {

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

        IntersectionPointPlaneModelPair leftWall =
                measureWall(xyzIj, devicePose, colorTdepthPose, 3, 3, 4, true);

        IntersectionPointPlaneModelPair rightWall =
                measureWall(xyzIj, devicePose, colorTdepthPose, 3, 3, 4, false);

        if (leftWall == null || rightWall == null) {
            return null;
        }

        // Update the AR object location.
        TangoPoseData leftPlaneFitPose =
                calculatePlanePose(extrinsics, leftWall.intersectionPoint, leftWall.planeModel,
                        devicePose);

        TangoPoseData rightPlaneFitPose =
                calculatePlanePose(extrinsics, rightWall.intersectionPoint, rightWall.planeModel,
                        devicePose);

        WallMeasurement left = new WallMeasurement(leftPlaneFitPose, devicePose);
        WallMeasurement right = new WallMeasurement(rightPlaneFitPose, devicePose);

        CornerMeasurement cornerMeasurement = null;
        if (Math.abs(getAngleBetweenPlanes(leftWall.planeModel, rightWall.planeModel)) >
                MAX_DEVIATION) {
            cornerMeasurement = new CornerMeasurement(left, right);
        }


        return cornerMeasurement;
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
                                                        int minCommons, boolean left) {

        List<IntersectionPointPlaneModelPair> candidates =
                getCandidates(xyzIj, devicePose, colorTdepthPose, horizontals, verticals, left);

        if (candidates.size() < minCommons) {
            return null;
        }

        List<IntersectionPointPlaneModelPair> winners = getWinners(candidates, MAX_DEVIATION);
        if (winners.size() < minCommons) {
            return null;
        }
        return getAverage(winners);
    }

    private List<IntersectionPointPlaneModelPair> getCandidates(TangoXyzIjData xyzIj,
                                                                TangoPoseData devicePose,
                                                                TangoPoseData colorTdepthPose,
                                                                int horizontals, int verticals,
                                                                boolean left) {
        List<IntersectionPointPlaneModelPair> candidates = new ArrayList<>();

        float hStep = 0.5f / (horizontals + 1.0f);
        float vStep = 1.0f / (verticals + 1.0f);
        float start = left ? 0.0f : 0.5f;
        float end = left ? 0.5f : 1.0f;

        if (left) {
            status.clear();
        }
        for (float x = start + hStep; x < end; x += hStep) {
            for (float y = vStep; y < 1.0f; y += vStep) {
                try {
                    IntersectionPointPlaneModelPair candidate = TangoSupport
                            .fitPlaneModelNearClick(xyzIj, mIntrinsics, colorTdepthPose, x, y);
                    if (isAlignedWithGravity(mExtrinsics, candidate, devicePose, MAX_DEVIATION)) {
                        candidates.add(candidate);
                        status.addCircle(x, y, true);
                    } else {
                        status.addCircle(x, y, false);
                    }
                } catch (TangoException | SecurityException t) {
                }
            }
        }
        if (!left) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    status.invalidate();
                }
            });
        }
        return candidates;
    }

    private boolean isAlignedWithGravity(DeviceExtrinsics extrinsics,
                                         IntersectionPointPlaneModelPair candidate,
                                         TangoPoseData devicePose, double maxDeviation) {
        Matrix4 adfTdevice = ScenePoseCalculator.tangoPoseToMatrix(devicePose);
        Vector3 gravityVector = ScenePoseCalculator.TANGO_WORLD_UP.clone();
        adfTdevice.clone().multiply(extrinsics.getDeviceTDepthCamera()).inverse().
                rotateVector(gravityVector);

        double[] gravity = new double[]{gravityVector.x, gravityVector.y, gravityVector.z};
        double angle = VectorUtilities.getAngleBetweenVectors(candidate.planeModel, gravity);

        double target = Math.PI / 2; // vectors should be perpendicular => 90° => PI / 2 in radians
        return (Math.abs(target - angle) <= maxDeviation);
    }

    private List<IntersectionPointPlaneModelPair> getWinners(
            List<IntersectionPointPlaneModelPair> candidates, double maxDeviation) {
        List<IntersectionPointPlaneModelPair> result = new ArrayList<>();

        boolean[][] candidatePairs = findPairs(candidates, maxDeviation);
        int index = getIndexOfMostPairs(candidates, candidatePairs);
        if (index > -1) {
            for (int i = 0; i < candidates.size(); i++) {
                if (candidatePairs[index][i]) {
                    result.add(candidates.get(i));
                }
            }
        }
        return result;
    }

    private int getIndexOfMostPairs(List<IntersectionPointPlaneModelPair> candidates,
                                    boolean[][] candidatePairs) {
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
        return index;
    }

    private boolean[][] findPairs(List<IntersectionPointPlaneModelPair> candidates,
                                  double maxDeviation) {
        boolean[][] result = new boolean[candidates.size()][candidates.size()];
        for (int i = 0; i < candidates.size(); i++) {
            for (int j = i + 1; j < candidates.size(); j++) {
                double angle = VectorUtilities.getAngleBetweenVectors(candidates.get(i).planeModel,
                        candidates.get(j).planeModel);
                if (angle < maxDeviation) {
                    result[i][j] = true;
                    result[j][i] = true;
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
    private TangoPoseData calculatePlanePose(DeviceExtrinsics extrinsicses, double[] point,
                                             double normal[], TangoPoseData devicePose) {
        Matrix4 adfTdevice = ScenePoseCalculator.tangoPoseToMatrix(devicePose);
        // Vector aligned to gravity.
        Vector3 depthUp = ScenePoseCalculator.TANGO_WORLD_UP.clone();
        adfTdevice.clone().multiply(extrinsicses.getDeviceTDepthCamera()).inverse().
                rotateVector(depthUp);
        // Create the plane matrix transform in depth frame from a point, the plane normal and the
        // up vector.
        Matrix4 depthTplane = ScenePoseCalculator.matrixFromPointNormalUp(point, normal, depthUp);
        // Plane matrix transform in adf frame.
        Matrix4 adfTplane =
                adfTdevice.multiply(extrinsicses.getDeviceTDepthCamera()).multiply(depthTplane);

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
        mFloorplan = PlanBuilder.buildPlan(mCornerMeasurementList, closed);
        mRenderer.updatePlan(mFloorplan);
    }

    /**
     * Updates every saved measurement. It re-queries the device pose at the time the measurement
     * was taken.
     */
    public void updateMeasurements() {
        mRenderer.removeCornerMeasurements();
        for (CornerMeasurement cornerMeasurement : mCornerMeasurementList) {
            TangoPoseData newDevicePose =
                    mTango.getPoseAtTime(cornerMeasurement.wallMeasurement.getDevicePoseTimeStamp(),
                            FRAME_PAIR);
            cornerMeasurement.update(newDevicePose);
            mRenderer.addCornerMeasurement(cornerMeasurement);
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

    private class Circle {
        public float x;
        public float y;
        public int radius;
        public Paint paint;

        public Circle(float x, float y, int radius, Paint paint) {
            this.x = x;
            this.y = y;
            this.radius = radius;
            this.paint = paint;
        }
    }

    private class StatusView extends View {
        private Paint mCandidatePaint;
        private Paint mNoCandidatePaint;
        private int RADIUS = 15;
        private int ALPHA = 50;
        private Collection<Circle> circles = new ArrayList<>();

        public StatusView(Context context) {
            super(context);
            mCandidatePaint = new Paint();
            mCandidatePaint.setStyle(Paint.Style.FILL);
            mCandidatePaint.setStrokeWidth(3);
            mCandidatePaint.setColor(Color.GREEN);
            mCandidatePaint.setAlpha(ALPHA);

            mNoCandidatePaint = new Paint();
            mNoCandidatePaint.setStyle(Paint.Style.FILL);
            mNoCandidatePaint.setStrokeWidth(3);
            mNoCandidatePaint.setColor(Color.RED);
            mNoCandidatePaint.setAlpha(ALPHA);
        }

        @Override
        protected void onDraw(Canvas canvas) {
            super.onDraw(canvas);
            synchronized (this) {
                for (Circle circle : circles) {
                    canvas.drawCircle(circle.x, circle.y, circle.radius, circle.paint);
                }
            }
        }

        public void clear() {
            circles.clear();
        }

        public void addCircle(float x, float y, boolean isCandidate) {
            Paint paint = isCandidate ? mCandidatePaint : mNoCandidatePaint;
            int width = getWidth();
            int height = getHeight();
            synchronized (this) {
                circles.add(new Circle(width * x, height * y, RADIUS, paint));
            }
        }
    }
}
