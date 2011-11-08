package team.scan;

import java.util.ArrayList;
import java.lang.Math;

public class LineFitter {

    private ArrayList<double[]> laserPoints;
    
    public LineFitter() {
	this.laserPoints = new ArrayList<double[]>();
	System.out.println("Creating new line fitter");
    }

    public void setPoints(ArrayList<double[]> points) {
	this.laserPoints = new ArrayList<double[]>();
	for (double[] p : points)
	    this.laserPoints.add(p);
    }

    public double[] getCentroid() {

	double[] sum = new double[]{0,0};

	sum[0] = this.getMomentX() / laserPoints.size();
	sum[1] = this.getMomentY() / laserPoints.size();

	return sum;

    }

    public double getMomentX() {
	double sum = 0;
	for (double[] pnt : this.laserPoints) {
	    sum += pnt[0];
	}
	return sum;
    }

    public double getMomentY() {
	double sum = 0;
	for (double[] pnt : this.laserPoints) {
	    sum += pnt[1];
	}
	return sum;
    }

    public double getMomentXY() {
	double sum = 0;
	for (double[] pnt : this.laserPoints) {
	    sum += pnt[0]*pnt[1];
	}
	return sum;
    }

    public double getMomentXX() {
	double sum = 0;
	for (double[] pnt : this.laserPoints) {
	    sum += pnt[0]*pnt[0];
	}
	return sum;
    }

    public double getMomentYY() {
	double sum = 0;
	for (double[] pnt : this.laserPoints) {
	    sum += pnt[1]*pnt[1];
	}
	return sum;
    }

    public double getCentroidXX() {
	int N = this.laserPoints.size();
	double cxx = getMomentXX()/N - Math.pow(getMomentX()/N, 2);
	return cxx;
    }

    public double getCentroidXY() {
	int N = this.laserPoints.size();
	double cxy = getMomentXY()/N - getMomentX()/N * getMomentY()/N;
	return cxy;
    }

    public double getCentroidYY() {
	int N = this.laserPoints.size();
	double cyy = getMomentYY()/N - Math.pow(getMomentX()/N, 2);
	return cyy;
    }

}
