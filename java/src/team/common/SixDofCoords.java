package team.common;

import april.jmat.LinAlg;
import april.jmat.Matrix;
import team.slam.Pose3D;

import java.util.*;

public class SixDofCoords {

    public static final double[] xOpenGlToHz = new double[]{0,0,0,Math.PI,0,0};

    public static void main(String[] args) {

        test();

    }

    //Tested with PeRL van toolbox in matlab
    private static void test() {

        Random rand = new Random(1337);

        double[] xij = new double[6];
        xij[0] = rand.nextDouble();
        xij[1] = rand.nextDouble();
        xij[2] = rand.nextDouble();
        xij[3] = rand.nextDouble();
        xij[4] = rand.nextDouble();
        xij[5] = rand.nextDouble();

        double[] xjk = new double[6];
        xjk[0] = rand.nextDouble();
        xjk[1] = rand.nextDouble();
        xjk[2] = rand.nextDouble();
        xjk[3] = rand.nextDouble();
        xjk[4] = rand.nextDouble();
        xjk[5] = rand.nextDouble();

        double[] xik = headToTail(xij, xjk);
        double[] xji = inverse(xij);
        double[] xjkEst = tailToTail(xij, xik);

        System.out.println("xij: ");
        ArrayUtil.print1dArray(xij);
        System.out.println();

        System.out.println("xjk: ");
        ArrayUtil.print1dArray(xjk);
        System.out.println();

        System.out.println("xik = xij \\oplus xjk:");
        ArrayUtil.print1dArray(xik);
        System.out.println();

        System.out.println("xji = \\ominus xij:");
        ArrayUtil.print1dArray(xji);
        System.out.println();

        System.out.println("xjkEst = tail2tail(xij, xik):");
        ArrayUtil.print1dArray(xjkEst);
        System.out.println();

        System.out.println("xjkEst Position:");
        ArrayUtil.print1dArray(getPosition(xjkEst));
        System.out.println();

        double[][] J = headToTailJacob(xij, xjk);
        System.out.println("Head to Tail Jacobian:");
        ArrayUtil.print2dArray(J);
        System.out.println();

        System.out.println("Inverse Jacobian:");
        J = inverseJacob(xij);
        ArrayUtil.print2dArray(J);
        System.out.println();

        System.out.println("Tail to Tail Jacobian:");
        J = tailToTailJacob(xij, xik);
        ArrayUtil.print2dArray(J);
        System.out.println();

        System.out.println("Positional Covariance Example:");
        J = headToTailJacob(xij, xik);
        //Compute a 6x6 covariance projection, assuming isotropic, univariate noise
        double[][] sigma = getPositionCov(LinAlg.matrixAB(J, LinAlg.transpose(J)));
        ArrayUtil.print2dArray(sigma);
        System.out.println();

    }

    /**
     * xik = xij \oplus xjk
     */
    public static double[] headToTail(double[] xij, double[] xjk) {

        Matrix Hij = new Matrix(LinAlg.xyzrpyToMatrix(xij));
        Matrix Hjk = new Matrix(LinAlg.xyzrpyToMatrix(xjk));
        Matrix Hik = new Matrix(4,4);

        Hik = Hij.times(Hjk);

        double[] xik = LinAlg.matrixToXyzrpy(Hik.copyArray());

        return xik;

    }

    /**
     * xji = \ominus xij
     */
    public static double[] inverse(double[] xij) {

        Matrix Hij = new Matrix(LinAlg.xyzrpyToMatrix(xij));

        //Get the 3x3 rotation matrix
        Matrix Rji = new Matrix(Hij.copyArray(0, 0, 3, 3));
        Matrix tij = new Matrix(Hij.copyArray(0, 3, 3, 1));

        //Convert to inverse coordinate transform
        Matrix Rij = Rji.transpose();
        Matrix tji = Rij.times(tij).times(-1.0);

        Matrix Hji = new Matrix(4, 4);

        //Set 3x3 rotation matrix in Hji
        Hji.set(0, 0, Rij.get(0, 0)); Hji.set(0, 1, Rij.get(0, 1)); Hji.set(0, 2, Rij.get(0, 2));
        Hji.set(1, 0, Rij.get(1, 0)); Hji.set(1, 1, Rij.get(1, 1)); Hji.set(1, 2, Rij.get(1, 2));
        Hji.set(2, 0, Rij.get(2, 0)); Hji.set(2, 1, Rij.get(2, 1)); Hji.set(2, 2, Rij.get(2, 2));

        //Set 3x1 translation matrix in Hji
        Hji.set(0, 3, tji.get(0));
        Hji.set(1, 3, tji.get(1));
        Hji.set(2, 3, tji.get(2));

        //Set the bottom row as '0 0 0 1'
        Hji.set(3, 3, 1.0);

        return LinAlg.matrixToXyzrpy(Hji.copyArray());

    }

    /**
     * xjk = \ominus xij \oplus xik
     */
    public static double[] tailToTail(double[] xij, double[] xik) {

        return headToTail(inverse(xij), xik);

    }

    // /**
    //  * Utility transformation. Mostly so we don't have to change the functions that are
    //  * already in here.
    //  */
    // public static Pose3D doubleArrayToPose3D(double[] values) {

    //     if (values.length != 6) {
    //         System.out.println("Error! Wrong length vector to transform into Pose3D");
    //     }

    //     return new Pose3D(values);

    // }

    // /**
    //  * Utility transformation. Mostly so we don't have to change the functions that are
    //  * already in here.
    //  */
    // public static double[] pose3DToDoubleArray(Pose3D aPose) {

    //     return aPose.getArray();

    // }


    /**
     * Get the position for a pose (useful for landmark constraints)
     */
    public static double[] getPosition(double[] relPose) {
        double[] pos = new double[3];
        for (int i = 0; i < 3; i++) {
            pos[i] = new Double(relPose[i]);
        }
        return pos;
    }

    public static double[] get2DXYTfrom3DPose(double[] pose) {

        assert(pose.length == 6);

        double[] result = new double[3];

        result[0] = pose[0];
        result[1] = pose[1];
        result[2] = pose[5];

        return result;
    }

    /**
     * Compute positional uncertainty by marginalizing out orientation
     * uncertainty.  Useful for getting the covariance of a landmark
     * constraint
     */
    public static double[][] getPositionCov(double[][] sigmaFull) {

        assert((sigmaFull.length == 6 && sigmaFull[0].length == 6));

        double[][] sigmaPosition = new double[3][3];

        for (int row = 0; row < 3; row++)
            for (int col = 0; col < 3; col++)
                sigmaPosition[row][col] = sigmaFull[row][col];

        return sigmaPosition;

    }

    public static double[][] getPositionCov(Matrix sigmaFull) {
        return getPositionCov(sigmaFull.copyArray());
    }

    /**
     * Compute jacobian for headToTail (see Eustice pg 150)
     */
    public static double[][] headToTailJacob(double[] xij, double[] xjk) {

        Matrix J = new Matrix(6, 12);

        double[] xik = headToTail(xij, xjk);

        Matrix Hij = new Matrix(LinAlg.xyzrpyToMatrix(xij));
        Matrix _Rji = new Matrix(Hij.copyArray(0, 0, 3, 3));
        Matrix _Rij = _Rji.transpose();
        double[][] Rji = _Rji.copyArray();
        double[][] Rij = _Rij.copyArray();

        Matrix Hjk = new Matrix(LinAlg.xyzrpyToMatrix(xjk));
        Matrix _Rkj = new Matrix(Hjk.copyArray(0, 0, 3, 3));
        Matrix _Rjk = _Rkj.transpose();
        double[][] Rkj = _Rkj.copyArray();
        double[][] Rjk = _Rjk.copyArray();

        double R13 = Rji[0][2]; double R12 = Rji[0][1];
        double R23 = Rji[1][2]; double R22 = Rji[1][1];
        double R33 = Rji[2][2]; double R32 = Rji[2][1];

        double Rkj12 = Rkj[0][1]; double Rkj13 = Rkj[0][2];

        double Rij13 = Rij[0][2]; double Rij23 = Rij[1][2];

        double x_ij = xij[0]; double y_ij = xij[1]; double z_ij = xij[2];
        double x_jk = xjk[0]; double y_jk = xjk[1]; double z_jk = xjk[2];
        double x_ik = xik[0]; double y_ik = xik[1]; double z_ik = xik[2];

        double phi_ij = xij[3]; double theta_ij = xij[4]; double psi_ij = xij[5];
        double phi_jk = xjk[3]; double theta_jk = xjk[4]; double psi_jk = xjk[5];
        double phi_ik = xik[3]; double theta_ik = xik[4]; double psi_ik = xik[5];

        Matrix I = Matrix.identity(3,3);
        Matrix O = new Matrix(3,3);

        double[][] M = new double[3][3];
        M[0][0] = R13*y_jk - R12*z_jk; M[0][1] = (z_ik - z_ij)*cos(psi_ij); M[0][2] = -(y_ik - y_ij);
        M[1][0] = R23*y_jk - R22*z_jk; M[1][1] = (z_ik - z_ij)*sin(psi_ij); M[1][2] = (x_ik - x_ij);
        M[2][0] = R33*y_jk - R32*z_jk; M[2][1] = -x_jk*cos(theta_ij) - (y_jk * sin(phi_ij) + z_jk*cos(phi_ij))*sin(theta_ij); M[2][2] = 0;

        double[][] K1 = new double[3][3];
        K1[0][0] = cos(theta_ij)*cos(psi_ik - psi_ij)*sec(theta_ik); K1[0][1] = sin(psi_ik - psi_ij)*sec(theta_ik); K1[0][2] = 0;
        K1[1][0] = -cos(theta_ij)*sin(psi_ik - psi_ij); K1[1][1] = cos(psi_ik - psi_ij); K1[1][2] = 0;
        K1[2][0] = Rkj12*sin(phi_ik) + Rkj13*cos(phi_ik)*sec(theta_ik); K1[2][1] = sin(psi_ik - psi_ij)*tan(theta_ik); K1[2][2] = 1;

        double[][] K2 = new double[3][3];
        K2[0][0] = 1; K2[0][1] = sin(phi_ik - phi_jk)*tan(theta_ik); K2[0][2] = (R13*cos(psi_ik) + R23*sin(psi_ik))*sec(theta_ik);
        K2[1][0] = 0; K2[1][1] = cos(phi_ik - phi_jk); K2[1][2] = -cos(theta_jk)*sin(phi_ik - phi_jk);
        K2[2][0] = 0; K2[2][1] = sin(phi_ik - phi_jk)*sec(theta_ik); K2[2][2] = cos(theta_jk)*cos(phi_ik - phi_jk)*sec(theta_ik);

        J.set(0, 0, I.copyArray()); J.set(0, 3, M); J.set(0, 6, Rji); J.set(0, 9, O.copyArray());
        J.set(3, 0, O.copyArray()); J.set(3, 3, K1); J.set(3,6, O.copyArray()); J.set(3, 9, K2);

        return J.copyArray();

    }

    /**
     * Numerically compute jacobian for tailToTail
     */
    public static double[][] tailToTailJacob(double[] xij, double[] xik) {

        Matrix h2tJacob = new Matrix(headToTailJacob(inverse(xij), xik));
        double[][] invJacob = inverseJacob(xij);
        Matrix otherMat = new Matrix(12, 12, Matrix.SPARSE);

        for (int k = 6; k < 12; k++)
            otherMat.set(k, k, 1.0);

        for (int row = 0; row < 6; row++) {
            for (int col = 0; col < 6; col++) {
                otherMat.set(row, col, invJacob[row][col]);
            }
        }

        Matrix t2tJacob = h2tJacob.times(otherMat);

        return t2tJacob.copyArray();

    }

    /**
     * Numerically compute jacobian for inverse
     */
    public static double[][] inverseJacob(double[] xij) {

        Matrix J = new Matrix(6, 6);

        double[] xji = inverse(xij);

        Matrix Hij = new Matrix(LinAlg.xyzrpyToMatrix(xij));
        Matrix _Rji = new Matrix(Hij.copyArray(0, 0, 3, 3));
        Matrix _Rij = _Rji.transpose();
        double[][] Rji = _Rji.copyArray();
        double[][] Rij = _Rij.copyArray();

        double R13 = Rji[0][2]; double R12 = Rji[0][1]; double R11 = Rji[0][0];
        double R23 = Rji[1][2]; double R22 = Rji[1][1]; double R21 = Rji[1][0];
        double R33 = Rji[2][2]; double R32 = Rji[2][1]; double R31 = Rji[2][0];

        double x_ij = xij[0]; double y_ij = xij[1]; double z_ij = xij[2];
        double x_ji = xji[0]; double y_ji = xji[1]; double z_ji = xji[2];

        double phi_ij = xij[3]; double theta_ij = xij[4]; double psi_ij = xij[5];
        double phi_ji = xji[3]; double theta_ji = xji[4]; double psi_ji = xji[5];

        double[][] N = new double[3][3];
        N[0][0] = 0;     N[0][1] = -R31*(x_ij*cos(psi_ij) + y_ij*sin(psi_ij)) + z_ij*cos(theta_ij);             N[0][2] = R21*x_ij - R11*y_ij;
        N[1][0] = z_ji;  N[1][1] = -R32*(x_ij*cos(psi_ij) + y_ij*sin(psi_ij)) + z_ij*sin(theta_ij)*sin(phi_ij); N[1][2] = R22*x_ij - R12*y_ij;
        N[2][0] = -y_ji; N[2][1] = -R33*(x_ij*cos(psi_ij) + y_ij*sin(psi_ij)) + z_ij*sin(theta_ij)*cos(phi_ij); N[2][2] = R23*x_ij - R13*y_ij;

        double alpha = 1 / (1 - sq(R13));
        double[][] Q = new double[3][3];
        Q[0][0] = -R11; Q[0][1] = -R12*cos(phi_ij); Q[0][2] = R13*R33;
        Q[1][0] = R12*sqrt(1-sq(R13)); Q[1][1] = -R33*cos(psi_ij)*sqrt(1 - sq(R13)); Q[1][2] = R23 * sqrt(1-sq(R13));
        Q[2][0] = R13*R11; Q[2][1] = -R23*cos(psi_ij); Q[2][2] = -R33;

        for (int row = 0; row < 3; row++)
            for (int col = 0; col < 3; col++)
                Q[row][col] *= alpha;

        J.set(0, 0, _Rji.transpose().times(-1.0).copyArray());
        J.set(0, 3, N);
        J.set(3, 3, Q);

        return J.copyArray();

    }

    public static double cos(double x) {
        return Math.cos(x);
    }

    public static double sin(double x) {
        return Math.sin(x);
    }

    public static double tan(double x) {
        return Math.tan(x);
    }

    public static double sec(double x) {
        return 1/cos(x);
    }

    public static double sq(double x) {
        return x*x;
    }

    public static double sqrt(double x) {
        return Math.sqrt(x);
    }

}
