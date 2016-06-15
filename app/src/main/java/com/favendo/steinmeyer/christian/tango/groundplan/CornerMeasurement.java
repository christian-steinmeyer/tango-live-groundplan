package com.favendo.steinmeyer.christian.tango.groundplan;

import android.util.Log;

import com.google.atap.tangoservice.TangoPoseData;

import org.rajawali3d.math.Quaternion;
import org.rajawali3d.math.vector.Vector3;

import java.text.DecimalFormat;

/**
 * Representation of a corner by a vertex and two walls, given in form of a vector and two
 * quaternions.
 *
 * @author Christian Steinmeyer on 09.06.2016.
 */
public class CornerMeasurement {

    public Vector3 vertex;
    public Quaternion wall;
    public Quaternion otherWall;
    public TangoPoseData wallPoseData;
    public TangoPoseData otherWallPoseData;
    private double maxDeviation = Math.PI / 90;


    public CornerMeasurement(WallMeasurement wall, WallMeasurement otherWall) {
        vertex = wall.intersect(otherWall);
        wallPoseData = wall.getPlanePose();
        otherWallPoseData = otherWall.getPlanePose();
        double[] r1 = wall.getPlanePose().rotation;
        this.wall = new Quaternion(r1[3], r1[0], r1[1], r1[2]);
        double[] r2 = otherWall.getPlanePose().rotation;
        this.otherWall = new Quaternion(r2[3], r2[0], r2[1], r2[2]);
        // NOTE: Rajawali quaternions use a left-hand rotation around the axis convention.
    }

    public boolean isNeighbor(CornerMeasurement otherCorner) {
        return wall.angleBetween(otherCorner.wall) < maxDeviation ||
                (wall.angleBetween(otherCorner.otherWall) < maxDeviation ||
                        (otherWall.angleBetween(otherCorner.wall) < maxDeviation ||
                                (otherWall.angleBetween(otherCorner.otherWall) < maxDeviation)));
    }

    @Override
    public boolean equals(Object other) {
        if (other == null) {
            return false;
        }
        if (other == this) {
            return true;
        }
        if (!(other instanceof CornerMeasurement)) {
            return false;
        }
        CornerMeasurement otherCorner = (CornerMeasurement) other;

        double vertexAngle = Math.toRadians(angleBetweenIgnoringZ(vertex, otherCorner.vertex));

        boolean result = vertexAngle < maxDeviation &&
                ((wall.angleBetween(otherCorner.wall) < maxDeviation &&
                        otherWall.angleBetween(otherCorner.otherWall) < maxDeviation) ||
                        (wall.angleBetween(otherCorner.otherWall) < maxDeviation &&
                                otherWall.angleBetween(otherCorner.wall) < maxDeviation));
        return result;
    }

    public double angleBetweenIgnoringZ(final Vector3 v1, final Vector3 v2) {
        Vector3 vA = v1.clone();
        vA.z = 0;
        Vector3 vB = v2.clone();
        vB.z = 0;
        return vA.angle(vB);
    }

}

