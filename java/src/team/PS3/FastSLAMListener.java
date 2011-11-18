/**
 * EECS 568 PS3, Task 1
 *
 * Steve Chaves
 * Schuyler Cohen
 * Patrick O'Keefe
 * Paul Ozog
 */

package team.PS3;

import java.awt.*;
import java.util.*;
import java.io.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;
import april.sim.*;
import java.util.Random;
import april.vis.VisCanvas.Movie;

public class FastSLAMListener implements OldSimulator.Listener
{

    VisWorld vw;
    Config config;
    Movie movie;

    private ArrayList<double[]> trajectory = new ArrayList<double[]>();

    private double baseline;
    private Particle bestChi2Particle;

    private int numUpdates = 0;
    private boolean debug = true;

    // The number of particles to use
    private int numParticles;

    // The list of current particles
    private ArrayList<Particle> particles;

    // The temporary list of particles that are updated and used for resampling
    private ArrayList<Particle> tempParticles;

    // Ivars for drawing
    private ArrayList<double[]> particlePositions = new ArrayList<double[]>();
    //private double[] bestChi2ParticleLocation;

    private double newFeatThreshold;

    private int randSeed = 0;
    private Random rand;

    String moviePath;

    public void init(Config _config, VisWorld _vw)
    {
        config  = _config;
        vw = _vw;

        baseline          = config.requireDouble("robot.baseline_m");
        Particle.baseline = baseline;
        newFeatThreshold  = config.requireDouble("fastSlam.threshold");

        double odomD[] = config.requireDoubles("noisemodels.odometryDiag");
        double featD[] = config.requireDoubles("noisemodels.landmarkDiag");
        numParticles   = config.requireInt("fastSlam.numParticles");
        randSeed       = config.requireInt("fastSlam.randSeed");
        moviePath      = config.requireString("fastSlam.moviePath");
        
        boolean autoframes = true;

        //movie = movieCreate(moviePath, true);

        rand = new Random(randSeed);
        Particle.setRandom(rand);
        //Create diagonal ticks L/R covariance matrix
        double[][] odomCov = new double[2][2];
        odomCov[0][0] = odomD[0]*odomD[0]; odomCov[0][1] = 0;
        odomCov[1][0] = 0;                 odomCov[1][1] = odomD[1]*odomD[1];

        double[][] featCov = new double[2][2];
        featCov[0][0] = featD[0]; featCov[0][1] = 0;
        featCov[1][0] = 0;        featCov[1][1] = featD[1];

        Particle.setSigmaW(new Matrix(featCov));
        Particle.setSigmaTicksLR(new Matrix(odomCov));

        Particle.setThreshold(newFeatThreshold);

        // Allocate enough space for our particles
        particles = new ArrayList<Particle>();
        for (int i = 0; i<numParticles; i++)
            particles.add(new Particle());

        tempParticles = new ArrayList<Particle>();
        for (int i = 0; i<numParticles; i++)
            tempParticles.add(new Particle());

    }



    public void update(OldSimulator.odometry_t odom, ArrayList<OldSimulator.landmark_t> dets)
    {

        numUpdates++;

        // Deep copy the current set of particles
        tempParticles = new ArrayList<Particle>();
        for(Particle p : particles){
            tempParticles.add(new Particle(p));
        }

        java.util.ArrayList<double[]> landmarkObs = new ArrayList<double[]>();
        for (OldSimulator.landmark_t det : dets) {
            landmarkObs.add(new double[]{det.obs[0], det.obs[1]});
        }

        /////////////////////
        // Particle Update
        /////////////////////

        // double totWeight = 0;
        double maxWeight = Double.NEGATIVE_INFINITY;
        for (Particle aParticle : tempParticles) {

            //TODO:  change xyt
            aParticle.updateParticleWithOdomAndObs(new double[]{odom.obs[0], odom.obs[1]}, landmarkObs);

            double curWeight = aParticle.getWeight();
            // totWeight += curWeight;
            maxWeight = maxWeight < curWeight ? curWeight : maxWeight;
        }


        //////////////////////
        // Particle resampling
        //////////////////////

        // Only resample if we have seen one or more landmarks
        if (!landmarkObs.isEmpty()) {
            particles.clear();
            resampleFromList(tempParticles,  maxWeight);
        } else {
            particles = new ArrayList<Particle>();
            for (Particle p : tempParticles) {
                particles.add(new Particle(p));
            }
        }


        assert(particles.size() == tempParticles.size());

        updateDrawingVariables();

        drawDummy(dets);

    }

    public void resampleFromList(ArrayList<Particle> tempList, double maxWeight) {
        System.out.println("Using max weight " +  maxWeight);
        double weights[] = new double[tempList.size()];
        double totWeight = 0;
        for(int i = 0; i < tempList.size(); i++){
            weights[i] = tempList.get(i).getWeight();
            // weights[i] = Math.exp(tempList.get(i).getWeight() - maxWeight);
            totWeight += weights[i];
        }
        System.out.println("Total Weight " + totWeight);
        for(int i = 0; i < weights.length; i++){
            weights[i] /= totWeight;
            //System.out.println("Weight i " + i + " " + weights[i]);
        }
        HashMap<Integer, Integer> count = new HashMap<Integer, Integer>();

        for(int i = 0; i < tempList.size(); i++){
            double pick = rand.nextDouble();
            double running = 0;
            for(int j = 0; j < weights.length; j++){
                /*silly hack to make sure double precision doesnt bite us*/
                if(running + weights[j] >= pick || (j + 1 == weights.length) ){
                    // System.out.println("Picked particle " + j);
                    if(count.containsKey(j)){
                        int val = count.get(j);
                        count.put(j, val + 1);
                    }
                    else{
                        count.put(j, 1);
                    }
                    particles.add(new Particle(tempList.get(j)));
                    break;
                }
                running += weights[j];
            }
        }

        int maxCount = 0;
        for(Map.Entry<Integer, Integer> entry : count.entrySet()){
            if(maxCount < entry.getValue()){
                maxCount = entry.getValue();
            }
            // System.out.println("Saw " + entry.getKey() + " " + entry.getValue() + " times.");
        }
        //System.out.println("Max count " + maxCount);

    }

    public void updateDrawingVariables() {

        double minChi2 = Double.POSITIVE_INFINITY;
        double [] mostLikelyPos = new double[3];

        particlePositions.clear();

        for (Particle aParticle : particles) {

            double[] pos = aParticle.getPoseXYT();

            particlePositions.add(new double[] {pos[0], pos[1]});

            // Keep track of the best particle's weight
            if (aParticle.getChi2() < minChi2) {
                minChi2 = aParticle.getChi2();
                //mostLikelyPos = aParticle.getPoseXYT();
                bestChi2Particle = aParticle;

            }

        }

	//Put the best particle's trajectory
	for (double[] xyt : bestChi2Particle.getCumStateXYT()) {
	    trajectory.add(LinAlg.resize(xyt,2));
	}

        //bestChi2ParticleLocation = new double[] {mostLikelyPos[0], mostLikelyPos[1], mostLikelyPos[2]};

    }

    public void drawDummy(ArrayList<OldSimulator.landmark_t> landmarks)
    {

        // Draw particle cloud
        {
            VisWorld.Buffer vb = vw.getBuffer("particle-cloud");

            //Draw a point at the XY of all particle poses
            vb.addBack(new VzPoints(new VisVertexData(particlePositions),
                                     new VisConstantColor(new Color(255,0,0)),
                                     2.0));

            //Draw a larger point at the XY of the most likely particle
            ArrayList<double[]> bestChi2ParticleLocationList =
                new ArrayList<double[]>(Arrays.asList(LinAlg.resize(bestChi2Particle.getPoseXYT(),2)));

            vb.addBack(new VzPoints(new VisVertexData(bestChi2ParticleLocationList),
                                     new VisConstantColor(new Color(0,255,0)),
                                     4.0));

            ArrayList<double[]> featureLocations = new ArrayList<double[]>();
            java.util.List<KalmanFeature> features = bestChi2Particle.getFeatures();
            for(KalmanFeature kf : features){
                featureLocations.add(kf.getLoc());
            }
            vb.addBack(new VzPoints(new VisVertexData(featureLocations), new VisConstantColor(new Color(111,111,111)), 10));
  
            vb.swap();
        }


 
        // Draw local Trajectory
        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-local");
            vb.addBack(new VzLines(new VisVertexData(trajectory),
                                    new VisConstantColor(new Color(160,30,30)),
                                    1.5, VzLines.TYPE.LINE_STRIP));
            vb.swap();
	    trajectory.clear();
        }

        // ArrayList<double[]> rpoints = new ArrayList<double[]>();
        // rpoints.add(new double[]{-.3, .3});
        // rpoints.add(new double[]{-.3, -.3});
        // rpoints.add(new double[]{.45,0});

        // Draws the robot triangle
        // {
        //     VisWorld.Buffer vb = vw.getBuffer("robot-local");
        //     VisObject robot = new VzLines(new VisVertexData(rpoints),
        //                                    new VisConstantColor(Color.red),
        //                                    3,
        //                                    VzLines.TYPE.LINE_LOOP);


        //     double xyzrpy[] = new double[]{xyt[0], xyt[1], 0,
        //                                    0, 0, xyt[2]};
        //     vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
        //     vb.swap();
        // }

        // Draw the landmark observations
        /*
        {
        
            VisWorld.Buffer vb = vw.getBuffer("landmarks-noisy");
            for (OldSimulator.landmark_t lmark : landmarks) {
                
                double[] obs = lmark.obs;
                
                ArrayList<double[]> obsPoints = new ArrayList<double[]>();
                obsPoints.add(LinAlg.resize(bestChi2ParticleLocation,2));
                double rel_xy[] = {obs[0] * Math.cos(obs[1]), obs[0] *Math.sin(obs[1])};
                obsPoints.add(LinAlg.transform(bestChi2ParticleLocation, rel_xy));
                
                vb.addBack(new VzLines(new VisVertexData(obsPoints),
                                        new VisConstantColor(lmark.id == -1? Color.gray : Color.cyan), 2, VzLines.TYPE.LINE_STRIP));
            }
          
            //vb.swap();
        }
        */

    }
}
