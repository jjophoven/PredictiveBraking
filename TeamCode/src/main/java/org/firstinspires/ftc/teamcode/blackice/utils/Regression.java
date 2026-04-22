package org.firstinspires.ftc.teamcode.blackice.utils;

import java.util.List;

public class Regression {
    /**
     * Slope of a best-fit line through origin (no intercept term).
     */
    public static double linearFit(double[][] points) {
        if (points == null || points.length == 0) {
            throw new IllegalArgumentException("Points array is empty");
        }

        double sumXY = 0;
        double sumXX = 0;

        for (double[] p : points) {
            double x = p[0];
            double y = p[1];
            sumXY += x * y;
            sumXX += x * x;
        }
        if (Math.abs(sumXX) < 1e-9) return 0;
        return sumXY / sumXX;
    }

    /**
     * Least-squares quadratic through origin (returns {b, a, r2, rmse}).
     * b = linear coefficient, a = quadratic coefficient, r2 = R-squared value, rmse = Root Mean Square Error
     */
    public static double[] quadraticFit(List<double[]> data) {
        if (data == null || data.isEmpty()) {
            throw new IllegalArgumentException("Data list is empty");
        }

        double sumV2 = 0;
        double sumV3 = 0;
        double sumV4 = 0;
        double sumVD = 0;
        double sumV2D = 0;
        double sumD = 0;
        double sumD2 = 0;

        for (double[] entry : data) {
            double v = entry[0];
            double d = entry[1];
            double v2 = v * v;
            sumV2 += v2;
            sumV3 += v2 * v;
            sumV4 += v2 * v2;
            sumVD += v * d;
            sumV2D += v2 * d;
            sumD += d;
            sumD2 += d * d;
        }

        double A11 = sumV2;
        double A12 = sumV3;
        double A21 = sumV3;
        double A22 = sumV4;

        double B1 = sumVD;
        double B2 = sumV2D;

        double det = A11 * A22 - A12 * A21;
        if (Math.abs(det) < 1e-9) return new double[]{0, 0, 0, 0};

        double b = (B1 * A22 - B2 * A12) / det;
        double a = (A11 * B2 - A21 * B1) / det;

        // Calculate R²
        int n = data.size();
        double meanD = sumD / n;
        double ssTotal = sumD2 - n * meanD * meanD;

        double ssResidual = 0;
        for (double[] entry : data) {
            double v = entry[0];
            double d = entry[1];
            double predicted = b * v + a * v * v;
            double residual = d - predicted;
            ssResidual += residual * residual;
        }

        double r2 = (ssTotal > 1e-9) ? 1 - (ssResidual / ssTotal) : 0;
        
        // Calculate RMSE
        double rmse = Math.sqrt(ssResidual / n);

        return new double[]{b, a, r2, rmse};
    }
}
