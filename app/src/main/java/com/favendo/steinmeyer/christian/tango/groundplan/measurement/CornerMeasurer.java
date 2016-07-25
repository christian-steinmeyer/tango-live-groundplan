package com.favendo.steinmeyer.christian.tango.groundplan.measurement;

import com.favendo.steinmeyer.christian.tango.groundplan.status.IStatusUpdater;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.projecttango.rajawali.DeviceExtrinsics;
import com.projecttango.rajawali.ScenePoseCalculator;
import com.projecttango.tangosupport.TangoSupport;
import com.projecttango.tangosupport.TangoSupport.IntersectionPointPlaneModelPair;

import org.rajawali3d.math.Matrix4;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector3;

import java.util.ArrayList;
import java.util.List;

public class CornerMeasurer {
    List<CornerMeasurement> mCornerMeasurementList;
    IStatusUpdater mStatusUpdater;
    DeviceExtrinsics mExtrinsics;
    TangoCameraIntrinsics mIntrinsics;

    // Accuracy
    private static final double MAX_DEVIATION = Math.PI / 90; // = 2°


    public CornerMeasurer(IStatusUpdater statusUpdater) {
        mCornerMeasurementList = new ArrayList<>();
        mStatusUpdater = statusUpdater;
    }

    public void setup(DeviceExtrinsics extrinsics, TangoCameraIntrinsics intrinsics) {
        mExtrinsics = extrinsics;
        mIntrinsics = intrinsics;
    }

    public CornerMeasurement measureForWalls(TangoXyzIjData xyzIj, TangoPoseData devicePose, double rgbTimestamp) {
        // Synchronize against concurrent access to the RGB timestamp in the
        // OpenGL thread and a possible service disconnection due to an onPause
        // event.
        CornerMeasurement cornerMeasurement;
        synchronized (this) {
            cornerMeasurement =
                    doCornerMeasurement(xyzIj, devicePose,
                            rgbTimestamp);
        }

        // If the measurement was successful add it and run the floor plan building
        // algorithm.
        if (cornerMeasurement != null) {
            if (isNewCorner(cornerMeasurement)) {
                add(cornerMeasurement);
                return cornerMeasurement;
            }
        }
        return null;
    }

    /**
     * Use the TangoSupport library and point cloud data to calculate the plane at the specified
     * location in the color camera frame. It returns the pose of the fitted plane in a
     * TangoPoseData structure.
     */
    public CornerMeasurement doCornerMeasurement(TangoXyzIjData xyzIj, TangoPoseData devicePose,
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

        IntersectionPointPlaneModelPair leftWall =
                measureWall(xyzIj, devicePose, colorTdepthPose, 3, 3, 4, true);

        IntersectionPointPlaneModelPair rightWall =
                measureWall(xyzIj, devicePose, colorTdepthPose, 3, 3, 4, false);

        if (leftWall == null || rightWall == null) {
            return null;
        }

        // Update the AR object location.
        TangoPoseData leftPlaneFitPose =
                calculatePlanePose(leftWall.intersectionPoint, leftWall.planeModel, devicePose);

        TangoPoseData rightPlaneFitPose =
                calculatePlanePose(rightWall.intersectionPoint, rightWall.planeModel, devicePose);

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
     * This method returns an {@link IntersectionPointPlaneModelPair} that is based on
     * the originally touched point, but also takes its environment into account. It will
     * additionally consider <code>accuracy</code> number of points and tries to fit an average
     * plane for which only the ones closer than 5% to the original one are considered.
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

        List<IntersectionPointPlaneModelPair> winners =
                getWinners(candidates, MAX_DEVIATION);
        if (winners.size() < minCommons) {
            return null;
        }
        return getAverage(winners);
    }

    private List<IntersectionPointPlaneModelPair> getCandidates(TangoXyzIjData xyzIj,
                                                                             TangoPoseData
                                                                                     devicePose,
                                                                             TangoPoseData
                                                                                     colorTdepthPose,
                                                                             int horizontals,
                                                                             int verticals,
                                                                             boolean left) {
        List<IntersectionPointPlaneModelPair> candidates = new ArrayList<>();

        float hStep = 0.5f / (horizontals + 1.0f);
        float vStep = 1.0f / (verticals + 1.0f);
        float start = left ? 0.0f : 0.5f;
        float end = left ? 0.5f : 1.0f;

        if (left) {
            mStatusUpdater.clear();
        }
        for (float x = start + hStep; x < end; x += hStep) {
            for (float y = vStep; y < 1.0f; y += vStep) {
                try {
                    IntersectionPointPlaneModelPair candidate = TangoSupport
                            .fitPlaneModelNearClick(xyzIj, mIntrinsics, colorTdepthPose, x, y);
                    if (isAlignedWithGravity(candidate, devicePose, MAX_DEVIATION)) {
                        candidates.add(candidate);
                        mStatusUpdater.addCircle(x, y, true);
                    } else {
                        mStatusUpdater.addCircle(x, y, false);
                    }
                } catch (TangoException | SecurityException t) {
                    t.printStackTrace();
                }
            }
        }
        if (!left) {
            mStatusUpdater.invalidate();
        }
        return candidates;
    }

    public boolean isNewCorner(CornerMeasurement newCorner) {
        for (CornerMeasurement corner : mCornerMeasurementList) {
            if (corner.equals(newCorner)) {
                return false;
            }
        }
        return true;
    }

    public double getAngleBetweenPlanes(double[] orientation, double[] other) {

        Quaternion wallOrientation =
                new Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]);
        Quaternion newWallOrientation = new Quaternion(other[0], other[1], other[2], other[3]);
        return wallOrientation.angleBetween(newWallOrientation);
    }

    public boolean isAlignedWithGravity(IntersectionPointPlaneModelPair candidate,
                                        TangoPoseData devicePose, double maxDeviation) {
        Matrix4 adfTdevice = ScenePoseCalculator.tangoPoseToMatrix(devicePose);
        Vector3 gravityVector = ScenePoseCalculator.TANGO_WORLD_UP.clone();
        adfTdevice.clone().multiply(mExtrinsics.getDeviceTDepthCamera()).inverse().
                rotateVector(gravityVector);

        double[] gravity = new double[]{gravityVector.x, gravityVector.y, gravityVector.z};
        double angle = VectorUtilities.getAngleBetweenVectors(candidate.planeModel, gravity);

        double target = Math.PI / 2; // vectors should be perpendicular => 90° => PI / 2 in radians
        return (Math.abs(target - angle) <= maxDeviation);
    }

    public List<IntersectionPointPlaneModelPair> getWinners(
            List<IntersectionPointPlaneModelPair> candidates, double maxDeviation) {
        List<IntersectionPointPlaneModelPair> result =
                new ArrayList<>();

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

    public int getIndexOfMostPairs(List<IntersectionPointPlaneModelPair> candidates,
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

    public boolean[][] findPairs(List<IntersectionPointPlaneModelPair> candidates,
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

    public IntersectionPointPlaneModelPair getAverage(
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
        return new IntersectionPointPlaneModelPair(intersectionPoint, planeModel);
    }

    /**
     * Calculate the pose of the plane based on the position and normal orientation of the plane and
     * align it with gravity.
     */
    public TangoPoseData calculatePlanePose(double[] point, double normal[],
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

    public void add(CornerMeasurement measurement) {
        mCornerMeasurementList.add(measurement);
    }

    public List<CornerMeasurement> getMeasurements() {
        return mCornerMeasurementList;
    }
}