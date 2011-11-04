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
    private int numParticles = 50;

    // The list of current particles
    private ArrayList<Particle> particles;

    // The temporary list of particles that are updated and used for resampling
    private ArrayList<Particle> tempParticles;

    private double newFeatThreshold;
  


    public void init(Config _config, VisWorld _vw)
    {
        config  = _config;
        vw = _vw;

        baseline         = config.requireDouble("robot.baseline_m");
        newFeatThreshold = config.requireDouble("fastSlam.threshold");
        Particle.setThreshold(newFeatThreshold);

        // Allocate enough space for our particles
        particles = new ArrayList<Particle>(numParticles);
        tempParticles = new ArrayList<Particle>(numParticles);

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


        // Copy the current set of particles...is this a deep copy?
        tempParticles = new ArrayList<Particle>(particles);

        java.util.List<double[]> landmarkObs = new ArrayList<double[]>();
        for (Simulator.landmark_t det : dets) {
            landmarkObs.add(new double[]{det.obs[0], det.obs[1]});
        }


        for (Particle aParticle : tempParticles) {

            aParticle.updateParticleWithOdomAndObs(new double[]{odom.obs[0], odom.obs[1]}, landmarkObs);
        }

        //TODO: Resample from tempParticles to update particles


        drawDummy(dets);

    }

    public void drawDummy(ArrayList<Simulator.landmark_t> landmarks)
    {
        // Draw local Trajectory
        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-local");
            vb.addBack(new VisLines(new VisVertexData(trajectory),
                                    new VisConstantColor(new Color(160,30,30)),
                                    1.5, VisLines.TYPE.LINE_STRIP));
            vb.swap();
        }

        ArrayList<double[]> rpoints = new ArrayList<double[]>();
        rpoints.add(new double[]{-.3, .3});
        rpoints.add(new double[]{-.3, -.3});
        rpoints.add(new double[]{.45,0});

        // Probably should be replaced with student-code
        {
            VisWorld.Buffer vb = vw.getBuffer("robot-local");
            VisObject robot = new VisLines(new VisVertexData(rpoints),
                                           new VisConstantColor(Color.red),
                                           3,
                                           VisLines.TYPE.LINE_LOOP);


            double xyzrpy[] = new double[]{xyt[0], xyt[1], 0,
                                           0, 0, xyt[2]};
            vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
            vb.swap();
        }

        // Draw the landmark observations
        {
            VisWorld.Buffer vb = vw.getBuffer("landmarks-noisy");
            for (Simulator.landmark_t lmark : landmarks) {
                double[] obs = lmark.obs;
                ArrayList<double[]> obsPoints = new ArrayList<double[]>();
                obsPoints.add(LinAlg.resize(xyt,2));
                double rel_xy[] = {obs[0] * Math.cos(obs[1]), obs[0] *Math.sin(obs[1])};
                obsPoints.add(LinAlg.transform(xyt, rel_xy));
                vb.addBack(new VisLines(new VisVertexData(obsPoints),
                                        new VisConstantColor(lmark.id == -1? Color.gray : Color.cyan), 2, VisLines.TYPE.LINE_STRIP));
            }
            vb.swap();
        }

        // public void drawDummy(ArrayList<Simulator.landmark_t> landmarks)
        // {
        //     // Draw local Trajectory -- the red robot path -- our least squares "best guess"
        //     {
        //         VisWorld.Buffer vb = vw.getBuffer("trajectory-local");
        //         vb.addBack(new VisLines(new VisVertexData(trajectory),
        //                                 new VisConstantColor(new Color(160,30,30)),
        //                                 1.5, VisLines.TYPE.LINE_STRIP));
        //         vb.swap();
        //     }

        //     ArrayList<double[]> rpoints = new ArrayList<double[]>();
        //     rpoints.add(new double[]{-.3, .3});
        //     rpoints.add(new double[]{-.3, -.3});
        //     rpoints.add(new double[]{.45,0});

        //     // Draws the robot triangle at xyt
        //     {
        //         VisWorld.Buffer vb = vw.getBuffer("robot-local");
        //         VisObject robot = new VisLines(new VisVertexData(rpoints),
        //                                        new VisConstantColor(Color.red),
        //                                        3,
        //                                        VisLines.TYPE.LINE_LOOP);


        //         double xyzrpy[] = new double[]{xyt[0], xyt[1], 0,
        //                                        0, 0, xyt[2]};
        //         vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
        //         vb.swap();
        //     }

        //     // Draw the landmark observations -- the blue lines shooting out of the red bot
        //     {
        //         VisWorld.Buffer vb = vw.getBuffer("landmarks-noisy");
        //         for (Simulator.landmark_t lmark : landmarks) {
        //             double[] obs = lmark.obs;
        //             ArrayList<double[]> obsPoints = new ArrayList<double[]>();
        //             obsPoints.add(LinAlg.resize(xyt,2));
        //             double rel_xy[] = {obs[0] * Math.cos(obs[1]), obs[0] *Math.sin(obs[1])};
        //             obsPoints.add(LinAlg.transform(xyt, rel_xy));
        //             vb.addBack(new VisLines(new VisVertexData(obsPoints),
        //                                     new VisConstantColor(lmark.id == -1? Color.gray : Color.cyan), 2, VisLines.TYPE.LINE_STRIP));
        //         }
        //         vb.swap();
        //     }

    }
}
