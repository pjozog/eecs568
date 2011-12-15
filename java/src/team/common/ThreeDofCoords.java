package team.common;

import april.jmat.LinAlg;
import april.jmat.Matrix;
import team.slam.Pose3D;

import java.util.*;

public class ThreeDofCoords {

    /**
     * xik = xij \oplus xjk
     */
    public static double[] headToTail(double[] xij, double[] xjk) {

        double[] xij6 = xyzrphFromXyt(xij);
        double[] xjk6 = xyzrphFromXyt(xjk);
                
        double[] xik6 = SixDofCoords.headToTail(xij6, xjk6);

        return xytFromXyzrph(xik6);

    }

    /**
     * xji = \ominus xij
     */
    public static double[] inverse(double[] xij) {

        double[] xij6 = xyzrphFromXyt(xij);
        double[] xji6 = SixDofCoords.inverse(xij6);

        return xytFromXyzrph(xji6);

    }

    /**
     * xjk = \ominus xij \oplus xik
     */
    public static double[] tailToTail(double[] xij, double[] xik) {

        return headToTail(inverse(xij), xik);

    }

    /**
     * Get the position for a pose (useful for landmark constraints)
     */
    public static double[] getPosition(double[] relPose) {
        double[] pos = new double[2];
        for (int i = 0; i < 2; i++) {
            pos[i] = new Double(relPose[i]);
        }
        return pos;
    }

    /**
     * Compute positional uncertainty by marginalizing out orientation
     * uncertainty.  Useful for getting the covariance of a landmark
     * constraint
     */
    public static double[][] getPositionCov(double[][] sigmaFull) {

        assert((sigmaFull.length == 3 && sigmaFull[0].length == 3));

        double[][] sigmaPosition = new double[2][2];

        for (int row = 0; row < 2; row++)
            for (int col = 0; col < 2; col++)
                sigmaPosition[row][col] = sigmaFull[row][col];

        return sigmaPosition;

    }

    public static double[][] getPositionCov(Matrix sigmaFull) {
        return getPositionCov(sigmaFull.copyArray());
    }

    public static double[][] get3DofJacob(double[][] jacobFull) {
        
        double[][] J = new double[3][6];

        J[0][0] = jacobFull[0][0]; J[0][1] = jacobFull[0][1]; J[0][2] = jacobFull[0][5];
        J[1][0] = jacobFull[1][0]; J[1][1] = jacobFull[1][1]; J[1][2] = jacobFull[1][5];
        J[2][0] = jacobFull[5][0]; J[2][1] = jacobFull[5][1]; J[2][2] = jacobFull[5][5];

        J[0][3] = jacobFull[0][6]; J[0][7] = jacobFull[0][7]; J[0][5] = jacobFull[0][11];
        J[1][3] = jacobFull[1][6]; J[1][7] = jacobFull[1][7]; J[1][5] = jacobFull[1][11];
        J[2][3] = jacobFull[5][6]; J[2][7] = jacobFull[5][7]; J[2][5] = jacobFull[5][11];

        return J;

    }

    public static double[][] get3DofJacobInv(double[][] jacobFull) {
        
        double[][] J = new double[3][3];

        J[0][0] = jacobFull[0][0]; J[0][1] = jacobFull[0][1]; J[0][2] = jacobFull[0][5];
        J[1][0] = jacobFull[1][0]; J[1][1] = jacobFull[1][1]; J[1][2] = jacobFull[1][5];
        J[2][0] = jacobFull[5][0]; J[2][1] = jacobFull[5][1]; J[2][2] = jacobFull[5][5];

        return J;

    }

    /**
     * Compute jacobian for headToTail (see Eustice pg 150)
     */
    public static double[][] headToTailJacob(double[] xij, double[] xjk) {
        
        double[] xij6 = xyzrphFromXyt(xij);
        double[] xjk6 = xyzrphFromXyt(xjk);

        double[][] J6 = SixDofCoords.headToTailJacob(xij6, xjk6);

        return get3DofJacob(J6);

    }

    /**
     * Numerically compute jacobian for tailToTail
     */
    public static double[][] tailToTailJacob(double[] xij, double[] xik) {

        double[] xij6 = xyzrphFromXyt(xij);
        double[] xik6 = xyzrphFromXyt(xik);

        double[][] J6 = SixDofCoords.tailToTailJacob(xij6, xik6);

        return get3DofJacob(J6);

    }

    /**
     * Numerically compute jacobian for inverse
     */
    public static double[][] inverseJacob(double[] xij) {

        double[] xij6 = xyzrphFromXyt(xij);

        double[][] J6 = SixDofCoords.inverseJacob(xij6);

        return get3DofJacobInv(J6);

    }

    public static double[] xyzrphFromXyt(double[] x) {

        assert(x.length == 3);

        double[] x6Dof = new double[6];
        x6Dof[0] = x[0];
        x6Dof[1] = x[1];
        x6Dof[5] = x[2];

        x6Dof[2] = 0;
        x6Dof[3] = 0;
        x6Dof[4] = 0;

        return x6Dof;

    }

    public static double[] xytFromXyzrph(double[] x) {

        assert(x.length == 6);

        double[] x3Dof = new double[3];
        x3Dof[0] = x[0];
        x3Dof[1] = x[1];
        x3Dof[2] = x[5];

        return x3Dof;

    }

}
