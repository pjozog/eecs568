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
            this.points.add(new double[] {aPoint[0], aPoint[1]});
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
	//Find biggest/smallest x
	double biggestX = Double.NEGATIVE_INFINITY;
	double biggestY = Double.NEGATIVE_INFINITY;
	double smallestX = Double.POSITIVE_INFINITY;
	double smallestY = Double.POSITIVE_INFINITY;
	//... and biggest/smallest indeces
	int biggestXInd = -1;
	int biggestYInd = -1;
	int smallestXInd = -1;
	int smallestYInd = -1;

	//Iterate through all points
	for (int i = 0; i < points.size(); i++) {
	    if (points.get(i)[0] > biggestX) {
		biggestX = points.get(i)[0];
		biggestXInd = i;
	    }
	    if (points.get(i)[1] > biggestY) {
		biggestY = points.get(i)[1];
		biggestYInd = i;
	    }
	    if (points.get(i)[0] < smallestX) {
		smallestX = points.get(i)[0];
		smallestXInd = i;
	    }
	    if (points.get(i)[1] < smallestY) {
		smallestY = points.get(i)[1];
		smallestYInd = i;
	    }
	}

	//Which variation is bigger?  X or Y?
	double xVariation = biggestX - smallestX;
	double yVariation = biggestY - smallestY;
	ArrayList<double[]> finalPoints = new ArrayList<double[]>();

	if (yVariation > xVariation) {
	    finalPoints.add(points.get(smallestYInd));
	    finalPoints.add(points.get(biggestYInd));
	} else {
	    finalPoints.add(points.get(smallestXInd));
	    finalPoints.add(points.get(biggestXInd));
	}

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

