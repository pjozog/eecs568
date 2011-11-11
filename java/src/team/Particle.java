package team;

import java.util.List;
import java.util.ArrayList;
import april.jmat.*;
import java.util.Random;

public class Particle {

    // Every particle has its current state and a list of features
    // that are represented by a kalman filter mean and covariance

    // We may want to keep around the previous states, but it's not
    // necessary for this task. We just need to show the spread of
    // the point cloud over time.

    // Particle state in global XYT
    private double stateXYT[] = new double[3];
    private ArrayList<double[]> cumStateXYT = new ArrayList<double[]>();

    private static Random rand = null;

    // The list of features -- careful with deep copying
    List<KalmanFeature> featureList = new ArrayList<KalmanFeature>();

    // The weight will only be updated internally
    private double weight;

    // The chi2 is updated after every motion and every landmark
    // observation (except the first time a landmark is seen, which
    // has chi2 of 0)
    private double chi2;

    private static double threshold;

    public static double baseline;

    private static Matrix sigmaW = null;
    private static Matrix sigmaTicksLR = null;

    public static void setRandom(Random r){
        rand = r;
    }

    public static void setSigmaW(Matrix m){
        sigmaW = m;
    }

    public static void setSigmaTicksLR(Matrix m){
        sigmaTicksLR = m;
    }

    public static Matrix getSigmaW(){
        assert(sigmaW != null);
        return sigmaW;
    }
    public List<KalmanFeature> getFeatures(){
        return featureList;
    }

    // Default constructor...used when Simulator is just beginning
    public Particle() {
        cumStateXYT.add(new double[]{0,0,0});
        stateXYT[0] = 0;
        stateXYT[1] = 0;
        stateXYT[2] = 0;
        this.weight = 1.0;
        this.chi2   = 0.0;
    }

    /** copy constructor**/
    public Particle(Particle p){

        this.weight = 1.0;

        for(int i = 0; i < stateXYT.length; i++){
            stateXYT[i] = p.stateXYT[i];
        }
        for(KalmanFeature feature : p.featureList){
            featureList.add(new KalmanFeature(feature));
        }

        this.chi2 = p.getChi2();

        this.cumStateXYT = new ArrayList<double[]>();
        for(double[] aState : p.getCumStateXYT()) {
            this.cumStateXYT.add(new double[]{aState[0],aState[1],aState[2]});
        }

    }

    /**
     * Constructor used after resampling. We want deep copies of state and
     * feature here...let's make sure it's right.
     *
     * @param state -- length 3 in global XYT
     * @param features -- the features we're inheriting. NEED DEEP COPY.
     */
    public Particle(double[] state, List<KalmanFeature> features) {

        this.weight = 1.0;

        // With literals, is this a deep copy?
        stateXYT[0] = state[0];
        stateXYT[1] = state[1];
        stateXYT[2] = state[2];

        // Probably isn't a deep copy. Damn you Java.
        featureList = new ArrayList<KalmanFeature>();
        for(KalmanFeature k : features){
            featureList.add(new KalmanFeature(k));
        }

    }

    /**set threshold for new features**/
    public static void setThreshold(double t){
        threshold = t;
    }


    /**
     * Method called from FastSLAMListener with our new information. This will
     * update our particles global state and will perform data association and
     * Kalman updates on associated features.
     *
     * @param odom -- noisey ticks LR from listener
     * @param landObs -- List of RT data. Not sure if this will compile...
     */
    public void updateParticleWithOdomAndObs(double[] ticksLR, List<double[]> landObs) {
        assert(sigmaW != null);
        assert(rand != null);
        // Update our state estimate
        stateXYT = LinAlg.xytMultiply(stateXYT,
                                      this.sampleFromMotionModel(sigmaTicksLR.copyArray(), ticksLR));
        stateXYT[2] = MathUtil.mod2pi(stateXYT[2]);

        this.cumStateXYT.add(stateXYT);

        // Perform data correspondence and Kalman filter updates
        double maxWeightResult = Double.NEGATIVE_INFINITY;
        for (double[] obs : landObs) {
            double result = dataCorrespondenceAndUpdate(obs);
            if (result > maxWeightResult) {
                maxWeightResult = result;
            }
        }

        if (maxWeightResult > Double.NEGATIVE_INFINITY) {
            weight = maxWeightResult;
        }

    }

    public double[] sampleFromMotionModel(double[][] cov, double[] mean) {

        assert(rand != null);
        double[][] proportionalCov = new double[2][2];
        proportionalCov[0][0] = cov[0][0]*mean[0]*mean[0];
        proportionalCov[0][1] = 0;
        proportionalCov[1][0] = 0;
        proportionalCov[1][1] = cov[1][1]*mean[1]*mean[1];

        MultiGaussian mvg = new MultiGaussian(proportionalCov, mean);
        double[] ticksLR  = mvg.sample(rand);
        this.chi2        += mvg.chi2(ticksLR);
        double[] xyt      = new double[3];
        double dPhi       = Math.atan2(ticksLR[1] - ticksLR[0], baseline);

        xyt[0] = (ticksLR[0]+ticksLR[1])/2;
        xyt[1] = 0;
        xyt[2] = MathUtil.mod2pi(dPhi);

        return xyt;

    }

    /**
     * This will loop over all of the features and find the most likely feature
     * for correspondence. This will also update that feature's Kalman filter
     * after choosing it.
     *
     * @param landmarkObs -- RT of the observation
     */
    private double dataCorrespondenceAndUpdate(double[] landmarkObs) {
        assert(sigmaW != null);
        double maxLikelihood = Double.NEGATIVE_INFINITY;
        KalmanFeature possibleMatch = null;

        double minChi2 = Double.POSITIVE_INFINITY;

        for (KalmanFeature aFeature : featureList) {
        
            double potentialChi2 = aFeature.getCorrespChi2(landmarkObs, stateXYT);
            double likelihood = aFeature.calculateLikelihoodOfCorrespondence(landmarkObs, stateXYT, potentialChi2);
            // System.out.println("Likelihood: " + likelihood);
            if (likelihood > maxLikelihood) {
                maxLikelihood = likelihood;
                possibleMatch = aFeature;
                minChi2       = potentialChi2;
            }

        }

        // Now "possibleMatch" represents the most likely feature to use for association
        // BUT, we have to consider that it's a new feature. Compare the maxLikelihood to
        // our "new feature threshold". See lines 11 and 12 in ProbRob.

        // System.out.println("Maxlikelihood: " + maxLikelihood);

        if (maxLikelihood < threshold) {

            // Then we're going to treat it as a new feature

            double mean [] = FastSLAMMotionModel.predictedFeaturePosXY(stateXYT, landmarkObs);
            Matrix rw = new Matrix(FastSLAMMotionModel.jacobianRw(stateXYT, landmarkObs));

            Matrix cov = rw.times(sigmaW.timesTranspose(rw));

            possibleMatch = new KalmanFeature(mean, cov);

            featureList.add(possibleMatch);

            //Do not change this.chi2

            return threshold;

        } else {

            // Perform the Kalman Filter update only on this feature in the list
            // There's no need to do this if it's a new features (residuals are zero)
            possibleMatch.kalmanUpdate(landmarkObs, stateXYT);

            //Keep running total chi2
            this.chi2 += minChi2;

            return maxLikelihood;

        }



    }

    /**
     * Used in the FastSLAMListener for particle resampling
     *
     * @return weight
     */
    public double getWeight() {
        return weight;
    }

    /**
     * Get the running chi2 error
     */
    public double getChi2() {
        return this.chi2;
    }

    /**
     * Returns the state of the particle. Used in the display function
     * of FastSLAMListener.
     *
     * @return stateXYT
     */
    public double[] getPoseXYT() {
        return stateXYT;
    }

    public ArrayList<double[]> getCumStateXYT() {
        return this.cumStateXYT;
    }

}
