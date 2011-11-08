package team;

import april.jmat.*;

public class KalmanFeature {

    // Every feature is represented by a mean and covariance
    private double mean[] = new double[2];
    private Matrix covariance;

    public KalmanFeature(double[] startMean, Matrix startCov) {

        // Deep?
        mean[0] = startMean[0];
        mean[1] = startMean[1];


        covariance = startCov.copy();

    }

    public KalmanFeature(KalmanFeature k){
        for(int i = 0; i < mean.length; i++){
            mean[i] = k.mean[i];
        }
        covariance = k.covariance.copy();

    }

    /**
     * Method to update a feature's mean and covariance after an observation
     *
     * @param observation -- The direct RT observation
     * @param robotPose -- The pose from which the robot observed the feature
     */
    public void kalmanUpdate(double[] observation, double[] robotPose) {

        //We want to do the three Kalman update equations:
        //K = this.covariance*Jx'*inv(Jx*SigmaX*Jx' + sigmaW)
        //xprime = x + K*(residual);
        //this.covariance = this.covariance - K*Jx*this.covariance

        Matrix K            = new Matrix(2,2); //Kalman gain
        double[] residual   = new double[2];
        double[] predictObs = FastSLAMMotionModel.predictedFeaturePosRT(robotPose,
                                                                        this.mean);

        residual[0] = observation[0] - predictObs[0];
        residual[1] = observation[1] - predictObs[1];

        double[][] Jx = FastSLAMMotionModel.jacobianJx(robotPose, this.mean);
        Matrix JxMat = new Matrix(Jx);

        Matrix JxSigXJx = JxMat.times(this.covariance).times(JxMat.transpose());
        Matrix SigW     = Particle.getSigmaW();

        K = this.covariance.times(JxMat.transpose()).times(JxSigXJx.plus(SigW).inverse());
        double[] KResid = K.times(residual);

        this.mean[0] += KResid[0];
        this.mean[1] += KResid[1];

        assert(KResid.length == 2);

        this.covariance = this.covariance.minus(K.times(JxMat).times(this.covariance));

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
        double [] zj_hat = FastSLAMMotionModel.predictedFeaturePosRT(robotPose, this.mean);

        Matrix rw = new Matrix(FastSLAMMotionModel.jacobianRw(robotPose, observation));
        Matrix sigmaW = Particle.getSigmaW();
        Matrix Qj = sigmaW.plus(rw.times(covariance.timesTranspose(rw)));

        MultiGaussian mg = new MultiGaussian(Qj.copyArray(), zj_hat);
        // double w = 1.0/Math.sqrt(Qj.times(2*Math.PI).det()) * Math.exp(-0.5*mg.chi2(observation));

        // Let's work with log likelihood
        double w = -Math.log(Qj.times(2*Math.PI).det()) - 0.5*mg.chi2(observation);

        return w;
    }
}
