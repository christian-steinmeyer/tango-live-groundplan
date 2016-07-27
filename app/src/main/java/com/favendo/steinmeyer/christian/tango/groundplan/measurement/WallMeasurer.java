package com.favendo.steinmeyer.christian.tango.groundplan.measurement;

import com.favendo.steinmeyer.christian.tango.groundplan.status.IStatusUpdater;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.projecttango.rajawali.DeviceExtrinsics;
import com.projecttango.rajawali.ScenePoseCalculator;
import com.projecttango.tangosupport.TangoSupport;

import org.rajawali3d.math.Matrix4;
import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector3;

import java.util.ArrayList;
import java.util.List;

/**
 * This class contains all logic for detecting walls and handle measurements.
 * <p/>
 * Created by Christian Steinmeyer on 27.07.2016.
 */
public class WallMeasurer {

    List<WallMeasurement> mWallMeasurements;
    WallMeasurement mLastWallMeasurement;
    IStatusUpdater mStatusUpdater;
    DeviceExtrinsics mExtrinsics;
    TangoCameraIntrinsics mIntrinsics;

    public WallMeasurer(IStatusUpdater statusUpdater) {
        mWallMeasurements = new ArrayList<>();
        mStatusUpdater = statusUpdater;
    }

    public void setup(DeviceExtrinsics extrinsics, TangoCameraIntrinsics intrinsics) {
        mExtrinsics = extrinsics;
        mIntrinsics = intrinsics;
    }

    public void add(WallMeasurement measurement) {
        mWallMeasurements.add(measurement);
    }

    public List<WallMeasurement> getMeasurements() {
        return mWallMeasurements;
    }

    public double getAngleBetweenPlanes(double[] orientation, double[] other) {

        Quaternion wallOrientation =
                new Quaternion(orientation[0], orientation[1], orientation[2], orientation[3]);
        Quaternion newWallOrientation = new Quaternion(other[0], other[1], other[2], other[3]);
        wallOrientation.normalize();
        newWallOrientation.normalize();
        return wallOrientation.angleBetween(newWallOrientation);
    }

    public boolean isAlignedWithGravity(TangoSupport.IntersectionPointPlaneModelPair candidate,
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

    public int getIndexOfMostPairs(List<TangoSupport.IntersectionPointPlaneModelPair> candidates,
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

    public void findPairs(List<TangoSupport.IntersectionPointPlaneModelPair> candidates,
                          double maxDeviation,
                          boolean[][] candidatePairs) {
        for (int i = 0; i < candidates.size(); i++) {
            for (int j = i + 1; j < candidates.size(); j++) {
                double angle = VectorUtilities.getAngleBetweenVectors(candidates.get(i).planeModel,
                        candidates.get(j).planeModel);
                if (angle < maxDeviation) {
                    candidatePairs[i][j] = true;
                    candidatePairs[j][i] = true;
                }
            }
        }
    }

    public TangoSupport.IntersectionPointPlaneModelPair getAverage(
            List<TangoSupport.IntersectionPointPlaneModelPair> intersectionPointPlaneModelPairs) {

        double[] intersectionPoint = new double[3];
        double[] planeModel = new double[4];
        for (TangoSupport.IntersectionPointPlaneModelPair each : intersectionPointPlaneModelPairs) {
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
     *
     * @param point      a point that lies in the plane
     * @param normal     a vector that is perpendicular to the plane
     * @param devicePose the current device pose that matches this measurement
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

    public List<TangoSupport.IntersectionPointPlaneModelPair> getWinners(
            List<TangoSupport.IntersectionPointPlaneModelPair> candidates, double maxDeviation) {
        List<TangoSupport.IntersectionPointPlaneModelPair> result = new ArrayList<>();
        boolean[][] candidatePairs = new boolean[candidates.size()][candidates.size()];

        findPairs(candidates, maxDeviation, candidatePairs);

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

    public List<TangoSupport.IntersectionPointPlaneModelPair> getCandidates(TangoXyzIjData xyzIj,
                                                                            TangoPoseData devicePose,
                                                                            TangoPoseData colorTdepthPose,
                                                                            int horizontals,
                                                                            int verticals) {
        List<TangoSupport.IntersectionPointPlaneModelPair> candidates = new ArrayList<>();

        float hStep = 1f / (horizontals + 1f);
        float vStep = 1f / (verticals + 1f);

        mStatusUpdater.clear();
        for (float x = hStep; x < 1f; x += hStep) {
            for (float y = vStep; y < 1f; y += vStep) {
                try {
                    TangoSupport.IntersectionPointPlaneModelPair candidate = TangoSupport
                            .fitPlaneModelNearClick(xyzIj, mIntrinsics, colorTdepthPose, x, y);
                    if (isAlignedWithGravity(candidate, devicePose, 0.02)) {
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
        mStatusUpdater.invalidate();
        return candidates;
    }

    /**
     * This method returns an {@link TangoSupport.IntersectionPointPlaneModelPair} that is based on the
     * originally touched point, but also takes its environment into account. It will additionally
     * consider <code>accuracy</code> number of points and tries to fit an average plane for which
     * only the ones closer than 5% to the original one are considered.
     *
     * @param xyzIj           required to create planes
     * @param devicePose      required to create planes
     * @param colorTdepthPose required to create planes
     * @param horizontals     number of horizontal measure points (in one line)
     * @param verticals       number of vertical measure points (in one column)
     * @param minCommons      number of common measure points required for positive feedback     @return average of
     */
    public TangoSupport.IntersectionPointPlaneModelPair measureWall(TangoXyzIjData xyzIj,
                                                                    TangoPoseData devicePose,
                                                                    TangoPoseData colorTdepthPose,
                                                                    int horizontals, int verticals,
                                                                    int minCommons) {

        List<TangoSupport.IntersectionPointPlaneModelPair> candidates =
                getCandidates(xyzIj, devicePose, colorTdepthPose, horizontals, verticals);

        List<TangoSupport.IntersectionPointPlaneModelPair> winners = getWinners(candidates, 0.02);
        if (winners.size() < minCommons) {
            return null;
        }

        return getAverage(winners);
    }

    /**
     * Use the TangoSupport library and point cloud data to calculate the plane at the specified
     * location in the color camera frame. It returns the pose of the fitted plane in a
     * TangoPoseData structure.
     *
     * @param xyzIj depth information used for wall measurement
     * @param devicePose needed to put new wall into context
     * @param rgbTimestamp needed to put new wall into context
     */
    public WallMeasurement doWallMeasurement(TangoXyzIjData xyzIj, TangoPoseData devicePose,
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

        TangoSupport.IntersectionPointPlaneModelPair wall =
                measureWall(xyzIj, devicePose, colorTdepthPose, 3, 3, 4);
        if (wall == null) {
            return null;
        }


        // Update the AR object location.
        TangoPoseData planeFitPose =
                calculatePlanePose(wall.intersectionPoint, wall.planeModel, devicePose);

        return new WallMeasurement(planeFitPose, devicePose);
    }

    public boolean isNewWall(WallMeasurement newWall) {
        for (WallMeasurement wall : mWallMeasurements) {
            double angle = getAngleBetweenPlanes(newWall.getPlanePose().rotation,
                    wall.getPlanePose().rotation);
            if (angle < 0.05) {
                // newWall is known - is "new" only if not equal to last wall
                if (wall.equals(mLastWallMeasurement)) {
                    return false;
                }
            }
        }
        return true;
    }

    public WallMeasurement measureForWalls(TangoXyzIjData xyzIj, TangoPoseData devicePose,
                                           double rgbTimestamp) {
        // Synchronize against concurrent access to the RGB timestamp in the
        // OpenGL thread and a possible service disconnection due to an onPause
        // event.
        WallMeasurement wallMeasurement;
        synchronized (this) {
            wallMeasurement =
                    doWallMeasurement(xyzIj, devicePose,
                            rgbTimestamp);
        }

        // If the measurement was successful add it and run the floor plan building
        // algorithm.
        if (wallMeasurement != null) {
            if (isNewWall(wallMeasurement)) {
                mLastWallMeasurement = wallMeasurement;
                add(wallMeasurement);
                return wallMeasurement;
            }
        }
        return null;
    }
}
