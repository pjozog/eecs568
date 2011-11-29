package team.common;

import april.tag.CameraUtil;
import april.jmat.LinAlg;
import april.jmat.Matrix;

public class TagUtil {

    //Returns pose from TARGET TO CAMERA
    public static double[] getPose(double[][] H, double tagsize, double fx, double fy) {
            
        double M[][] = CameraUtil.homographyToPose(fx, fy, tagsize, H);

        //The CameraUtil gives us pose from camera to
        //target.  Also, the camera is using the messed-up
        //openGl-style camera frame.  So we need to get
        //the pose from target to camera, then convert to
        //Hartley-Zisserman camera convention (z forward,
        //x right, y down)
        double[] poseOpenGlCamToTag = LinAlg.matrixToXyzrpy(M);
        double[] poseTagToOpenGlCam = SixDofCoords.inverse(LinAlg.matrixToXyzrpy(M));
        double[] poseTagToHzCam     = SixDofCoords.headToTail(poseTagToOpenGlCam, SixDofCoords.xOpenGlToHz);
        return poseTagToHzCam;

    }

    //Returning a matrix so you can easily copyAsVector() the result
    public static Matrix getPoseSigma(double[][] H, double[][] HCov, double tagsize, double fx, double fy) {
        Matrix J = new Matrix(TagUtil.getPoseJacob(H, tagsize, fx, fy));
        Matrix Sigma = J.times(new Matrix(HCov)).times(J.transpose());
        return Sigma;
    }

    public static double[][] getPoseJacob(double[][] H, double tagsize, double fx, double fy) {
            
        Matrix HMat = new Matrix(H);
        double[] h = HMat.getColumnPackedCopy();

        double eps = .01;

        Matrix J = new Matrix(6, 9);

        //Differentiate with respect to h
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 6; j++) {

                double[] hPert = new double[h.length];
                for (int k = 0; k < h.length; k++)
                    hPert[k] = h[k];

                hPert[i] += eps;

                double[] y = getPose(hVecToMat(h), tagsize, fx, fy);
                double[] yPert = getPose(hVecToMat(hPert), tagsize, fx, fy);

                double finiteDiff = (yPert[j] - y[j]) / eps;

                J.set(j, i, finiteDiff);
                
            }
        }
            
        return J.copyArray();
            
    }    

    public static double[][] hVecToMat(double[] h) {
        return Matrix.columnPackedMatrix(h, 3, 3).copyArray();
    }

}
