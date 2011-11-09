package team.scan;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

public class Line {

    private List<double[]> points = null;
    private double[] centroid;
    private double theta;

    public Line(List<double[]> initialPoints, double[] aCentroid, double aTheta) {
        this.points = new ArrayList<double[]>(initialPoints);
        this.centroid = aCentroid;
        this.theta = aTheta;
    }

    public Line(Line one, Line two) {

        // Copy both sets of points
        // Make sure this is as deep as possible (that's what she said)
        this.points = new ArrayList<double[]>();
        for (double [] aPoint : one.getPoints()) {
            this.points.add(new double[] {aPoint[0], aPoint[1]});
        }

        for (double [] aPoint : two.getPoints()) {

            // Ensure we didn't already add this point to the list
            if (!one.getPoints().contains(aPoint)) {
                this.points.add(new double[] {aPoint[0], aPoint[1]});
            } else {

                // System.out.println("We found a duplicate point and ignored it");
            }

        }





        this.centroid = PointMoments.getCentroid(points);
        double Cxx        = PointMoments.getCentroidXX(points);
        double Cxy        = PointMoments.getCentroidXY(points);
        double Cyy        = PointMoments.getCentroidYY(points);
        this.theta        = Math.PI/2 + 0.5 * Math.atan2(-2*Cxy, Cyy - Cxx);

    }

    public Line(List<double[]> points) {

        double[] centroid = PointMoments.getCentroid(points);
        double Cxx        = PointMoments.getCentroidXX(points);
        double Cxy        = PointMoments.getCentroidXY(points);
        double Cyy        = PointMoments.getCentroidYY(points);
        double theta      = Math.PI/2 + 0.5 * Math.atan2(-2*Cxy, Cyy - Cxx);

        this.points = points;
        this.theta = Math.PI/2 + 0.5 * Math.atan2(-2*Cxy, Cyy - Cxx);
        this.centroid = centroid;

    }

    public ArrayList<double[]> getPointsForDisplay() {

        double[] extremeX = new double[]{Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        double[] extremeY = new double[]{Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};

        for (double[] aPoint : points) {

            // X
            if (aPoint[0] < extremeX[0]) {
                extremeX[0] = aPoint[0];
            }
            if (aPoint[0] > extremeX[1]) {
                extremeX[1] = aPoint[0];
            }
            // Y
            if (aPoint[1] < extremeY[0]) {
                extremeY[0] = aPoint[1];
            }
            if (aPoint[1] > extremeY[1]) {
                extremeY[1] = aPoint[1];
            }
        }

        // Find radiiiii
        double r1 = Math.sqrt(Math.pow(extremeX[1] - centroid[0], 2) +
                              Math.pow(extremeY[1] - centroid[1], 2));

        double r2 = Math.sqrt(Math.pow(extremeX[0] - centroid[0], 2) +
                              Math.pow(extremeY[0] - centroid[1], 2));

        double[] pointOne = new double[] {r1*Math.cos(theta), r1*Math.sin(theta)};
        double[] pointTwo = new double[] {-r2*Math.cos(theta), -r2*Math.sin(theta)};

        // Add the centroids to finish'er up

        for (int i = 0; i < centroid.length; i++) {
            pointOne[i] += centroid[i];
            pointTwo[i] += centroid[i];
        }


        ArrayList<double[]> finalPoints = new ArrayList<double[]>();
        finalPoints.add(pointOne);
        finalPoints.add(pointTwo);

        return finalPoints;


    }

    public double computeMSE() {

        int N        = points.size();
        double Cxx   = PointMoments.getCentroidXX(points);
        double Cxy   = PointMoments.getCentroidXX(points);
        double Cyy   = PointMoments.getCentroidXX(points);
        double nHatX = -Math.sin(theta);
        double nHatY = Math.cos(theta);
        double MSE   = Cxx*nHatX*nHatX/N + Cxy*nHatX*nHatY/N + Cyy*nHatY*nHatY/N;

        return MSE;

    }

    public List<double[]> getPoints() {
        return points;
    }


    public static List<Line> removeTwoPointLines(List<Line> origLines) {

        List<Line> newLines = new ArrayList<Line>();
        for (Line aLine : origLines) {
            if (aLine.getPoints().size() > 2) {
                newLines.add(aLine);
            }
        }

        return newLines;


    }

}

