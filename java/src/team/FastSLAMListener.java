/**
 * EECS 568 PS3, Task 1
 *
 * Steve Chaves
 * Schuyler Cohen
 * Patrick O'Keefe
 * Paul Ozog
 */

package team;

import java.awt.*;
import java.util.*;
import java.io.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;
import april.sim.*;
import java.util.Random;

public class FastSLAMListener implements Simulator.Listener
{

    VisWorld vw;
    Config config;


    private ArrayList<double[]> trajectory = new ArrayList<double[]>();
    double xyt[] = new double[3];

    private double baseline;


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
    private double[] mostLikelyParticleLocation;

    private double newFeatThreshold;

    private Random rand = new Random(1337);

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

        //Create diagonal ticks L/R covariance matrix
        double[][] odomCov = new double[2][2];
        odomCov[0][0] = odomD[0]*odomD[0]; odomCov[0][1] = 0;
        odomCov[1][0] = 0;                 odomCov[1][1] = odomD[1]*odomD[1];

        double[][] featCov = new double[2][2];
        featCov[0][0] = featD[0]*featD[0]; featCov[0][1] = 0;
        featCov[1][0] = 0;                 featCov[1][1] = featD[1]*featD[1];

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



    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets)
    {

        numUpdates++;

        {
            DenseVec ticksXYT = TicksUtil.ticksToXYT(odom, baseline);

            double x = ticksXYT.get(0);
            double y = ticksXYT.get(1);
            double t = ticksXYT.get(2);;


            xyt = LinAlg.xytMultiply(xyt, new double[]{(odom.obs[0] + odom.obs[1]) /2, 0,
                                                       Math.atan((odom.obs[1] - odom.obs[0])/baseline)});

            trajectory.add(LinAlg.resize(xyt,2));
        }

        // Deep copy the current set of particles
        tempParticles = new ArrayList<Particle>();
        for(Particle p : particles){
            tempParticles.add(new Particle(p));
        }

        java.util.ArrayList<double[]> landmarkObs = new ArrayList<double[]>();
        for (Simulator.landmark_t det : dets) {
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

        particles.clear();

        resampleFromList(tempParticles,  maxWeight);

        assert(particles.size() == tempParticles.size());

        updateDrawingVariables();

        drawDummy(dets);

    }

    public void resampleFromList(ArrayList<Particle> tempList, double maxWeight) {

        double weights[] = new double[tempList.size()];
        double totWeight = 0;
        for(int i = 0; i < tempList.size(); i++){
            //weights[i] = tempList.get(i).getWeight() / totalWeight;
            weights[i] = Math.exp(tempList.get(i).getWeight() - maxWeight);
            totWeight += weights[i];
        }

        for(int i = 0; i < weights.length; i++){
            weights[i] /= totWeight;
        }

        for(int i = 0; i < tempList.size(); i++){
            double pick = rand.nextDouble();
            double running = 0;
            for(int j = 0; j < weights.length; j++){
                /*silly hack to make sure double precision doesnt bite us*/
                if(running + weights[j] >= pick || (j + 1 == weights.length) ){
                    particles.add(new Particle(tempList.get(j)));
                    break;
                }
                running += weights[j];
            }
        }

    }

    public void updateDrawingVariables() {

        double maxWeight = Double.NEGATIVE_INFINITY;
        double [] mostLikelyPos = new double[3];

        particlePositions.clear();

        for (Particle aParticle : particles) {

            double[] pos = aParticle.getPoseXYT();

            particlePositions.add(new double[] {pos[0], pos[1]});

            // Keep track of the most likely particle's weight
            if (aParticle.getWeight() > maxWeight) {
                maxWeight = aParticle.getWeight();
                mostLikelyPos = aParticle.getPoseXYT();
            }

        }

        mostLikelyParticleLocation = new double[] {mostLikelyPos[0], mostLikelyPos[1], mostLikelyPos[2]};

    }

    public void drawDummy(ArrayList<Simulator.landmark_t> landmarks)
    {

        // Draw particle cloud
        {
            VisWorld.Buffer vb = vw.getBuffer("particle-cloud");

            //Draw a point at the XY of all particle poses
            vb.addBack(new VisPoints(new VisVertexData(particlePositions),
                                     new VisConstantColor(new Color(255,0,0)),
                                     2.0));

            //Draw a larger point at the XY of the most likely particle
            ArrayList<double[]> mostLikelyParticleLocationList =
                new ArrayList<double[]>(Arrays.asList(LinAlg.resize(mostLikelyParticleLocation,2)));
            vb.addBack(new VisPoints(new VisVertexData(mostLikelyParticleLocationList),
                                     new VisConstantColor(new Color(0,255,0)),
                                     4.0));

            vb.swap();
        }



        // Draw local Trajectory
        // {
        //     VisWorld.Buffer vb = vw.getBuffer("trajectory-local");
        //     vb.addBack(new VisLines(new VisVertexData(trajectory),
        //                             new VisConstantColor(new Color(160,30,30)),
        //                             1.5, VisLines.TYPE.LINE_STRIP));
        //     vb.swap();
        // }

        // ArrayList<double[]> rpoints = new ArrayList<double[]>();
        // rpoints.add(new double[]{-.3, .3});
        // rpoints.add(new double[]{-.3, -.3});
        // rpoints.add(new double[]{.45,0});

        // Draws the robot triangle
        // {
        //     VisWorld.Buffer vb = vw.getBuffer("robot-local");
        //     VisObject robot = new VisLines(new VisVertexData(rpoints),
        //                                    new VisConstantColor(Color.red),
        //                                    3,
        //                                    VisLines.TYPE.LINE_LOOP);


        //     double xyzrpy[] = new double[]{xyt[0], xyt[1], 0,
        //                                    0, 0, xyt[2]};
        //     vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
        //     vb.swap();
        // }

        // Draw the landmark observations
        {
            VisWorld.Buffer vb = vw.getBuffer("landmarks-noisy");
            for (Simulator.landmark_t lmark : landmarks) {
                double[] obs = lmark.obs;
                ArrayList<double[]> obsPoints = new ArrayList<double[]>();
                obsPoints.add(LinAlg.resize(mostLikelyParticleLocation,2));
                double rel_xy[] = {obs[0] * Math.cos(obs[1]), obs[0] *Math.sin(obs[1])};
                obsPoints.add(LinAlg.transform(mostLikelyParticleLocation, rel_xy));
                vb.addBack(new VisLines(new VisVertexData(obsPoints),
                                        new VisConstantColor(lmark.id == -1? Color.gray : Color.cyan), 2, VisLines.TYPE.LINE_STRIP));
            }
            vb.swap();
        }

    }
}
