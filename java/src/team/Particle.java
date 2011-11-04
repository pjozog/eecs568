package team;

import java.util.List;
import java.util.ArrayList;

public class Particle {

    // Every particle has its current state and a list of features
    // that are represented by a kalman filter mean and covariance

    // We may want to keep around the previous states, but it's not
    // necessary for this task. We just need to show the spread of
    // the point cloud over time.

    // Particle state in global XYT
    double stateXYT[] = new double[3];

    // The list of features -- careful with deep copying
    List<KalmanFeature> featureList = new ArrayList<KalmanFeature>();

    // The weight will only be updated internally
    private double weight;


    // Default constructor...used when Simulator is just beginning
    public Particle() {
        stateXYT[0] = 0;
        stateXYT[1] = 0;
        stateXYT[2] = 0;
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
        featureList = new ArrayList<KalmanFeature>(features);

    }

    /**
     * Method called from FastSLAMListener with our new information. This will
     * update our particles global state and will perform data association and
     * Kalman updates on associated features.
     *
     * @param odom -- EITHER XYT or RT...whatever we find more convenient here
     * @param landObs -- List of RT data. Not sure if this will compile...
     */
    public void updateParticleWithOdomAndObs(double[] odom, List<double[]> landObs) {

        // Update our state estimate
        sampleNewPoseFromOdom(odom);

        // Perform data correspondence and Kalman filter updates
        for (double[] obs : landObs) {
            dataCorrespondenceAndUpdate(obs);
        }

    }

    /**
     * The only method that touches ivar stateXYT. This is just a direct
     * application of our motion model to this particle's current state.
     *
     * @param odom -- EITHER XYT or RT...whatever we find more convenient here
     */
    private void sampleNewPoseFromOdom(double[] odom) {

        // Apply motion model. Add to current state.
    }

    /**
     * This will loop over all of the features and find the most likely feature
     * for correspondence. This will also update that feature's Kalman filter
     * after choosing it.
     *
     * @param landmarkObs -- RT of the observation
     */
    private void dataCorrespondenceAndUpdate(double[] landmarkObs) {

        double maxLikelihood = Double.NEGATIVE_INFINITY;
        KalmanFeature possibleMatch;

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

        if (maxLikelihood < SOMETHRESHOLD) {

            // Then we're going to treat it as a new feature
            possibleMatch = new KalmanFeature(MEAN, COV);

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

}
