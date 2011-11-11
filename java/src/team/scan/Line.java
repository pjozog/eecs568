package team.scan;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;
import team.ArrayUtil;

public class Line {

    private List<double[]> points = null;
    private double[] centroid;
    private double theta;
    public static double pointDistanceThreshold;

    public Line(List<double[]> initialPoints, double[] aCentroid, double aTheta) {
        this.points = new ArrayList<double[]>(initialPoints);
        this.centroid = aCentroid;
        this.theta = fullThetaToLimitedTheta(aTheta);
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
        this.theta        = fullThetaToLimitedTheta(Math.PI/2 + 0.5 * Math.atan2(-2*Cxy, Cyy - Cxx));


    }

    public static Line mergeLines(Line one, Line two) {
        if (shouldConsiderMergingLines(one, two)) {
            return new Line(one, two);
        } else {
            return null;
        }
    }


    public static Line initialLine(List<double[]> points) {
        // Point sanity check
        double[] pointOne = points.get(0);
        double[] pointTwo = points.get(1);
        double dist = Math.pow(pointOne[0] - pointTwo[0],2) + Math.pow(pointOne[1]-pointTwo[1],2);

        if (dist < pointDistanceThreshold) {
            return new Line(points);
        } else {
            return null;
        }

    }

    public Line(List<double[]> points) {

        double[] centroid = PointMoments.getCentroid(points);
        double Cxx        = PointMoments.getCentroidXX(points);
        double Cxy        = PointMoments.getCentroidXY(points);
        double Cyy        = PointMoments.getCentroidYY(points);
        double theta      = Math.PI/2 + 0.5 * Math.atan2(-2*Cxy, Cyy - Cxx);

        this.points = points;
        this.centroid = centroid;
        this.theta = fullThetaToLimitedTheta( Math.PI/2 + 0.5 * Math.atan2(-2*Cxy, Cyy - Cxx));

    }

    public double getTheta(){
        return theta;
    }

    public double [] getCentroid(){
        return centroid;
    }

    /**
     * This will look at the end points of both lines and compute the distances between
     * all four possibilities. If the shortest of the four exceeds our pointDistanceThreshold,
     * then we should not even attempt to merge the lines.
     */
    public static boolean shouldConsiderMergingLines(Line one, Line two) {

        // UGLY CODE. Sorry Schuyler.
        List<double[]> lOnePoints = one.getPoints();
        List<double[]> lTwoPoints = two.getPoints();
        double[] lOneStart = lOnePoints.get(0);
        double[] lOneEnd = lOnePoints.get(lOnePoints.size()-1);
        double[] lTwoStart = lTwoPoints.get(0);
        double[] lTwoEnd = lTwoPoints.get(lTwoPoints.size()-1);

        double smallestDistance = euclidianDistanceBetweenPoints(lOneStart, lTwoStart);

        double tempDist = euclidianDistanceBetweenPoints(lOneStart, lTwoEnd);
        if (tempDist < smallestDistance) {
            smallestDistance = tempDist;
        }

        tempDist = euclidianDistanceBetweenPoints(lOneEnd, lTwoStart);
        if (tempDist < smallestDistance) {
            smallestDistance = tempDist;
        }

        tempDist = euclidianDistanceBetweenPoints(lOneEnd, lTwoEnd);
        if (tempDist < smallestDistance) {
            smallestDistance = tempDist;
        }


        return (smallestDistance < pointDistanceThreshold) ? true : false;

    }

    public ArrayList<double[]> getPointsForDisplay() {

        double[] extremeX = new double[]{Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        double[] extremeY = new double[]{Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};

        // System.out.println("Point Locations");
        for (double[] aPoint : points) {

            // ArrayUtil.print1dArray(aPoint);

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


        // System.out.println("Extremes");
        // ArrayUtil.print1dArray(extremeX);
        // ArrayUtil.print1dArray(extremeY);

        // Find radiiiii
        double r1 = Math.sqrt(Math.pow(extremeX[1] - centroid[0], 2) +
                              Math.pow(extremeY[1] - centroid[1], 2));

        double r2 = Math.sqrt(Math.pow(extremeX[0] - centroid[0], 2) +
                              Math.pow(extremeY[0] - centroid[1], 2));

        // System.out.println("Radiiiii: " + r1 + "   " +r2);



        double[] pointOne = new double[] {r1*Math.cos(theta), r1*Math.sin(theta)};
        double[] pointTwo = new double[] {-r2*Math.cos(theta), -r2*Math.sin(theta)};

        // System.out.println("Centroid");
        // ArrayUtil.print1dArray(centroid);


        // Add the centroids to finish'er up

        for (int i = 0; i < centroid.length; i++) {
            pointOne[i] += centroid[i];
            pointTwo[i] += centroid[i];
        }

        // System.out.println("FinalPoints");
        // ArrayUtil.print1dArray(pointOne);
        // ArrayUtil.print1dArray(pointTwo);


        ArrayList<double[]> finalPoints = new ArrayList<double[]>();
        finalPoints.add(pointOne);
        finalPoints.add(pointTwo);

        return finalPoints;


    }

    public double computeMSE() {

        //duplicate points, and center at origin
        ArrayList<double[]> dupPoints = new ArrayList<double[]>();
        for (double[] point : this.points) {
            dupPoints.add(new double[]{point[0]-centroid[0], point[1]-centroid[1]});
        }

        int N        = points.size();
        double Mxx   = PointMoments.getMomentXX(dupPoints);
        double Mxy   = PointMoments.getMomentXY(dupPoints);
        double Myy   = PointMoments.getMomentYY(dupPoints);
        double nHatX = -Math.sin(theta);
        double nHatY = Math.cos(theta);
        double MSE   = Mxx*nHatX*nHatX + Mxy*nHatX*nHatY + Myy*nHatY*nHatY;

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

    // Ensures that the parameter reprsents an angle between -pi/2 to pi/2
    private double fullThetaToLimitedTheta(double theta) {

        if (theta > Math.PI/2) {
            return theta - Math.PI;
        } else if (theta < - Math.PI/2) {
            return theta + Math.PI;
        } else {
            return theta;
        }

    }

    private static double euclidianDistanceBetweenPoints(double[] one, double[] two) {
        return  Math.sqrt(Math.pow(one[0] - two[0], 2) +
                          Math.pow(one[1] - two[1], 2));
    }

}
