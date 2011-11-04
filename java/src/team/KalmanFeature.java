package team;

import april.jmat.*;

public class KalmanFeature {

    // Every feature is represented by a mean and covariance
    private double mean[] = new double[3];
    private Matrix covariance;

    public KalmanFeature(double[] startMean, Matrix startCov) {

        // Deep?
        mean[0] = startMean[0];
        mean[1] = startMean[1];
        mean[2] = startMean[2];

        covariance = startCov.copy();

    }

    public KalmanFeature(KalmanFeature k){
        for(int i = 0; i < mean.length; i++){
            mean[i] = k.mean[i];
        }
        covariance = k.covariance.copy();
        
    }

    /**
     * Static method to update a feature's mean and covariance after an observation
     *
     * @param observation -- The direct RT observation
     * @param robotPose -- The pose from which the robot observed the feature
     */
    public void kalmanUpdate(double[] observation, double[] robotPose) {
            

    }

    /**
     * Used in unknown data association. Each particle will loop over all of its
     * KalmanFeatures and choose the feature with the highest likelihood of
     * correspondence to perform the association.
     *
     * @param observation -- The direct RT observation
     * @param robotPose -- The pose from which the robot observed the feature
     *
     * @return the likelihood that the observation corresponds to this feature
     */
    public double calculateLikelihoodOfCorrespondence(double[] observation, double[] robotPose) {
        /*TODO figure out Z*/
       
        double [] zj_hat = FastSLAMMotionModel(robotPose, observation);

        Matrix rw = new Matrix(FastSLAMMotionModel.jacobianRw(robotPose, observation));
        Matrix sigmaW = Particle.getSigmaW();
        Matrix Qj = covariance.plus rw.times(sigmaW.timesTranspose(rw));
        
        MultiGaussian mg = MultiGaussian(Qj.copyArray(), zj.copyArray());
        double w = Math.sqrt(Qj.times(2*Math.PI).det()) * Math.exp(-1/2*mg.chi2(zj_hat));
        return w;
    }
}
