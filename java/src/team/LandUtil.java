package team;

import april.jmat.*;

public class LandUtil{

    /**Converts a relative reading to a landmark, (r theta) to global coordinates based on 
      the current state of the robot (x y p), all in radians
      math is Lx_global = r*cos(theta + phi) + Robot_x_global
              Ly_global = r*sin(theta + phi) + Robot_y_global**/
    public static double [] rThetaToXY(double r, double theta, double x, double y, double p){
	double [] pos = new double[2];
	pos[0] = r * Math.cos(theta + p) + x;
	pos[1] = r * Math.sin(theta + p) + y;
	return pos;

    }

}