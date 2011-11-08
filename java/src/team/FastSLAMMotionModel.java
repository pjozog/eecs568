package team;

import april.sim.*;
import april.jmat.*;
import java.lang.Math;

class FastSLAMMotionModel {

    public static double baseline;
    
    public static double[] predictedFeaturePosXY(double[] posObs, double[] rTheta) {
	
	double fx = posObs[0] + rTheta[0]*Math.cos(rTheta[1] + posObs[2]);
	double fy = posObs[1] + rTheta[0]*Math.sin(rTheta[1] + posObs[2]);

	double[] featPos = new double[2];

	featPos[0] = fx;
	featPos[1] = fy;

	return featPos;

    }

    public static double[] predictedFeaturePosRT(double[] posXYT, double[] featXY) {
	
	double x = posXYT[0];
	double y = posXYT[1];
	double t = posXYT[2];

	double fx = featXY[0];
	double fy = featXY[1];

	double r = Math.sqrt(Math.pow(fx-x,2) + Math.pow(fy-y,2));

	double theta = MathUtil.mod2pi(Math.atan2(fy-y,fx-x) - t);

	double[] rangeTheta = new double[2];
	rangeTheta[0] = r;
	rangeTheta[1] = theta;

	return rangeTheta;

    }

    public static double[][] jacobianRw(double[] posXYT, double[] rTheta) {

	//index by jacob[rol][col]
	double[][] jacob = new double[2][2];

	double r     = rTheta[0];
	double theta = rTheta[1];
	double poseT = posXYT[2];


	jacob[0][0] =    Math.cos(poseT + theta); 
	jacob[0][1] = -r*Math.cos(poseT + theta);

	jacob[1][0] =    Math.sin(poseT + theta); 
	jacob[1][1] =  r*Math.cos(poseT + theta);

	return jacob;

    }

    public static double[][] jacobianJx(double[] posXYT, double[] featPos) {

	double l_x = featPos[0];
	double l_y = featPos[1];
	double x0  = posXYT[0];
	double y0  = posXYT[1];

	//index by jacob[rol][col]
        double[][] jacob = new double[2][2];
        jacob[0][0]      =   (2*l_x - 2*x0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        jacob[0][1]      =   (2*l_y - 2*y0)/(2*Math.sqrt(Math.pow(l_x - x0,2) + Math.pow(l_y - y0,2)));
        jacob[1][0]      =   -(l_y - y0)/(Math.pow(l_x - x0,2)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));
        jacob[1][1]      =   1/((l_x - x0)*(Math.pow(l_y - y0,2)/Math.pow(l_x - x0,2) + 1));

	return jacob;

    }

    public static double[][] jacobianJw() {
	
	return Matrix.identity(2, 2).copyArray();

    }

    //XYT is RELATIVE
    public static DenseVec ticksToXYT(Simulator.odometry_t odom, double baseline) {

	return TicksUtil.ticksToXYT(odom, baseline);

    }

}
