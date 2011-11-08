package team.scan;

import java.util.ArrayList;
import java.util.List;


public class Line {

    private List<double[]> points = null;
    private double[] centroid;
    private double theta;

    public Line(List<double[]> initialPoints, double[] aCentroid, double aTheta) {
        this.points = new ArrayList<double[]>(initialPoints);
        this.centroid = aCentroid;
        this.theta = aTheta;
    }

    public Line(Line one, Line two, double[] aCentroid, double aTheta) {

        // Copy both sets of points
        // Make sure this is as deep as possible (that's what she said)
        this.points = new ArrayList<double[]>();
        for (double [] aPoint : one.getPoints()) {
            this.points.add(new double[] {aPoint[0], aPoint[1]});
        }
        for (double [] aPoint : two.getPoints()) {
            this.points.add(new double[] {aPoint[0], aPoint[1]});
        }

        this.centroid = aCentroid;
        this.theta = aTheta;

    }

    public List<double[]> getPointsForDisplay() {

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
            if (aPoint[0] < extremeY[0]) {
                extremeY[0] = aPoint[0];
            }
            if (aPoint[0] > extremeY[1]) {
                extremeY[1] = aPoint[0];
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


        List<double[]> finalPoints = new ArrayList<double[]>();
        finalPoints.add(pointOne);
        finalPoints.add(pointTwo);

        return finalPoints;


    }

    public double computeMSE() {

		int N        = points.size();
		double Mxx   = PointMoments.getMomentXX(points);
		double Mxy   = PointMoments.getMomentXX(points);
		double Myy   = PointMoments.getMomentXX(points);
		double nHatX = -Math.sin(theta);
		double nHatY = Math.cos(theta);
		double MSE   = Mxx*nHatX*nHatX/N + Mxy*nHatX*nHatY/N + Myy*nHatY*nHatY/N;
		
		return MSE;

    }

    public List<double[]> getPoints() {
        return points;
    }

}

