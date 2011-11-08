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

    //The covariance of ticks left/right
    public static double[][] ticksCov;

    public static Random rand = new Random();

    // The list of features -- careful with deep copying
    List<KalmanFeature> featureList = new ArrayList<KalmanFeature>();

    // The weight will only be updated internally
    private double weight;

    private static double threshold;

    public static double baseline;

    private static Matrix sigmaW = null;

    public static void setSigmaW(Matrix m){
        sigmaW = m;
    }
    public static Matrix getSigmaW(){
        assert(sigmaW != null);
        return sigmaW;
    }
    // Default constructor...used when Simulator is just beginning
    public Particle() {
        stateXYT[0] = 0;
        stateXYT[1] = 0;
        stateXYT[2] = 0;
	this.weight = 1;
    }

    /** copy constructor**/
    public Particle(Particle p){

	this.weight = 1;

        for(int i = 0; i < stateXYT.length; i++){
            stateXYT[i] = p.stateXYT[i]; 
        }   
        for(KalmanFeature feature : p.featureList){
            featureList.add(new KalmanFeature(feature));
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

        // Update our state estimate
	stateXYT = LinAlg.xytMultiply(stateXYT, 
				      this.sampleFromMotionModel(this.ticksCov, ticksLR));

        // Perform data correspondence and Kalman filter updates
        // for (double[] obs : landObs) {
        //     dataCorrespondenceAndUpdate(obs);
        // }

    }

    public double[] sampleFromMotionModel(double[][] cov, double[] mean) {
	MultiGaussian mvg = new MultiGaussian(cov, mean);
	double[] ticksLR  = mvg.sample(rand);
	double[] xyt      = new double[3];
        double dPhi       = Math.atan2(ticksLR[1] - ticksLR[0], baseline);

	xyt[0] = (ticksLR[0]+ticksLR[1])/2;
        xyt[1] = 0;
        xyt[2] = dPhi;

	return xyt;
	
    }

    /**
     * This will loop over all of the features and find the most likely feature
     * for correspondence. This will also update that feature's Kalman filter
     * after choosing it.
     *
     * @param landmarkObs -- RT of the observation
     */
    private void dataCorrespondenceAndUpdate(double[] landmarkObs) {
        assert(sigmaW != null);
        double maxLikelihood = Double.NEGATIVE_INFINITY;
        KalmanFeature possibleMatch = null;

        for (KalmanFeature aFeature : featureList) {

            double likelihood = aFeature.calculateLikelihoodOfCorrespondence(landmarkObs, stateXYT);
            if (likelihood > maxLikelihood) {
                maxLikelihood = likelihood;
                possibleMatch = aFeature;
            }
        }

        // Now "possibleMatch" represents the most likely feature to use for association
        // BUT, we have to consider that it's a new feature. Compare the maxLikelihood to
        // our "new feature threshold". See lines 11 and 12 in ProbRob.

        if (maxLikelihood < threshold) {

            // Then we're going to treat it as a new feature
            
            double mean [] = FastSLAMMotionModel.predictedFeaturePosXY(stateXYT, landmarkObs);
            Matrix rw = new Matrix(FastSLAMMotionModel.jacobianJx(stateXYT, landmarkObs));
 
            Matrix cov = rw.times(sigmaW.timesTranspose(rw));

            possibleMatch = new KalmanFeature(mean, cov);

        } else {

            // Perform the Kalman Filter update only on this feature in the list
            // There's no need to do this if it's a new features (residuals are zero)
            possibleMatch.kalmanUpdate(landmarkObs, stateXYT);

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
     * Returns the state of the particle. Used in the display function
     * of FastSLAMListener.
     *
     * @return stateXYT
     */
    public double[] getPoseXYT() {
        return stateXYT;
    }

}
