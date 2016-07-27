package com.favendo.steinmeyer.christian.tango.groundplan.measurement;

import java.util.Locale;

/**
 * Some simple methods for operations on vectors, given as arrays of {@link double}s.
 *
 * @author Christian Steinmeyer on 30.05.2016.
 */
public class VectorUtilities {

    private VectorUtilities() {
        super();
    }

    public static String print(double[] vector) {
        if (vector.length < 1) {
            return "[]";
        }
        String result = "[";
        for (double each : vector) {
            result += String.format(Locale.GERMANY, "%1$,.3f", each) + ", ";
        }
        return result.substring(0, result.length() - 2) + "]";
    }

    public static double getLength(double[] vector) {
        double sum = 0.0;
        for (double dimension : vector) {
            sum += (dimension * dimension);
        }
        return Math.sqrt(sum);
    }

    public static double[] normalize(double[] planeModel) {
        double length = getLength(planeModel);
        double[] result = planeModel.clone();
        for (int i = 0; i < result.length; i++) {
            result[i] = result[i] / length;
        }
        return result;
    }


    /**
     * Calculates the angle between two planes according to http://www.wolframalpha
     * .com/input/?i=dihedral+angle
     */
    public static double getAngleBetweenVectors(double[] a, double[] b) {
        double numerator = 0;
        for (int i = 0; i < Math.min(a.length, b.length); i++) {
            numerator += a[i] * b[i];
        }
        double denominator = getLength(a) * getLength(b);
        return Math.acos(numerator / denominator);
    }
}
