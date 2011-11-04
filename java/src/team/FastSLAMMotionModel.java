package april.jmat;
import java.lang.Math;

class FastSLAMMotionModel {
    
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



}