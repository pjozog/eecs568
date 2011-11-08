package team.scan;

import java.util.ArrayList;
import java.lang.Math;

public class PointMoments {

    private ArrayList<double[]> points;
    
    public PointMoments() {
	this.points = new ArrayList<double[]>();
    }

    public void setPoints(ArrayList<double[]> points) {
	this.points = new ArrayList<double[]>();
	for (double[] p : points)
	    this.points.add(p);
    }

    public double[] getCentroid() {

	double[] sum = new double[]{0,0};

	sum[0] = this.getMomentX() / points.size();
	sum[1] = this.getMomentY() / points.size();

	return sum;

    }

    public double getMomentX() {
	double sum = 0;
	for (double[] pnt : this.points) {
	    sum += pnt[0];
	}
	return sum;
    }

    public double getMomentY() {
	double sum = 0;
	for (double[] pnt : this.points) {
	    sum += pnt[1];
	}
	return sum;
    }

    public double getMomentXY() {
	double sum = 0;
	for (double[] pnt : this.points) {
	    sum += pnt[0]*pnt[1];
	}
	return sum;
    }

    public double getMomentXX() {
	double sum = 0;
	for (double[] pnt : this.points) {
	    sum += pnt[0]*pnt[0];
	}
	return sum;
    }

    public double getMomentYY() {
	double sum = 0;
	for (double[] pnt : this.points) {
	    sum += pnt[1]*pnt[1];
	}
	return sum;
    }

    public double getCentroidXX() {
	int N = this.points.size();
	double cxx = getMomentXX()/N - Math.pow(getMomentX()/N, 2);
	return cxx;
    }

    public double getCentroidXY() {
	int N = this.points.size();
	double cxy = getMomentXY()/N - getMomentX()/N * getMomentY()/N;
	return cxy;
    }

    public double getCentroidYY() {
	int N = this.points.size();
	double cyy = getMomentYY()/N - Math.pow(getMomentX()/N, 2);
	return cyy;
    }

}
