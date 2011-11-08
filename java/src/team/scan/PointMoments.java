package team.scan;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

public class PointMoments {

    public static void setPoints(List<double[]> points) {
        points = new ArrayList<double[]>();
        for (double[] p : points)
            points.add(p);
    }

    public static double[] getCentroid(List<double[]> points) {

        double[] sum = new double[]{0,0};

        sum[0] = getMomentX(points) / points.size();
        sum[1] = getMomentY(points) / points.size();

        return sum;

    }

    public static double getMomentX(List<double[]> points) {
        double sum = 0;
        for (double[] pnt : points) {
            sum += pnt[0];
        }
        return sum;
    }

    public static double getMomentY(List<double[]> points) {
        double sum = 0;
        for (double[] pnt : points) {
            sum += pnt[1];
        }
        return sum;
    }

    public static double getMomentXY(List<double[]> points) {
        double sum = 0;
        for (double[] pnt : points) {
            sum += pnt[0]*pnt[1];
        }
        return sum;
    }

    public static double getMomentXX(List<double[]> points) {
        double sum = 0;
        for (double[] pnt : points) {
            sum += pnt[0]*pnt[0];
        }
        return sum;
    }

    public static double getMomentYY(List<double[]> points) {
        double sum = 0;
        for (double[] pnt : points) {
            sum += pnt[1]*pnt[1];
        }
        return sum;
    }

    public static double getCentroidXX(List<double[]> points) {
        int N = points.size();
        double cxx = getMomentXX(points)/N - Math.pow(getMomentX(points)/N, 2);
        return cxx;
    }

    public static double getCentroidXY(List<double[]> points) {
        int N = points.size();
        double cxy = getMomentXY(points)/N - getMomentX(points)/N * getMomentY(points)/N;
        return cxy;
    }

    public static double getCentroidYY(List<double[]> points) {
        int N = points.size();
        double cyy = getMomentYY(points)/N - Math.pow(getMomentX(points)/N, 2);
        return cyy;
    }

}
