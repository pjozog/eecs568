package team.sim;

import java.awt.Color;
import java.util.*;
import java.io.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;
import april.sim.*;

import team.common.*;
import team.slam.*;

import team.PS2.LandUtil;

public class Listener implements OldSimulator.Listener {

    private VisWorld vw;
    private Config config;
    private BackEnd slam;

    private double baseline;
    private double lambda;
    private double epsilon;

    private int numConverge;

    Pose3DNode prevPose;

    // Profiling
    private long startTime;
    private int numUpdates = 0;

    // Drawing
    ArrayList<VzLines> trajectory = new ArrayList<VzLines>();
    // ArrayList<VzPoints> landmarks = new ArrayList<VzPoints>();
    ArrayList<double[]> landmarks = new ArrayList<double[]>();
    ArrayList<double[]> allEdgeLinks = new ArrayList<double[]>();

    public void init(Config _config, VisWorld _vw) {

        config  = _config;
        vw = _vw;


        baseline    = config.requireDouble("robot.baseline_m");
        lambda      = config.requireDouble("simulator.lambda");
        lambda = 0.0;
        epsilon     = config.requireDouble("simulator.epsilon");

        numConverge = config.requireInt("simulator.numConverge");

        slam = new BackEnd(numConverge, lambda, epsilon);

        startTime = System.nanoTime();

        addPrior();
    }

    /**
     * Adds a Pose3DNode and a Pose3DEdge to the graph. This is needed so we don't have an
     * underconstrained systen.
     */
    public void addPrior() {


        Matrix cov = Matrix.identity(6, 6);
        // cov.times(100);

        // Create Pose3D at origin
        Pose3D p3d = new Pose3D();

        // Create Pose3DEdge
        Pose3DNode p3dn = new Pose3DNode();
        Pose3DEdge p3de = new Pose3DEdge(p3dn, p3d, cov);

        prevPose = p3dn;

        slam.addNode(p3dn);
        slam.addEdge(p3de);

    }

    public void update(OldSimulator.odometry_t odom, ArrayList<OldSimulator.landmark_t> dets) {

        numUpdates++;

        DenseVec ticksXYT = TicksUtil.ticksToXYT(odom, baseline);

        double x = ticksXYT.get(0);
        double y = ticksXYT.get(1);
        double t = ticksXYT.get(2);

        //Turn XYT into Pose3D
        Pose3D deltaMotion = new Pose3D(x, y, 0, 0, 0, t);

        Pose3DNode p3dn = new Pose3DNode();

        Matrix cov = Matrix.identity(6, 6);
        // cov.times(100);

        //Create Pose3DtoPose3DEdge with prevPose
        Pose3DToPose3DEdge poseToPose = new Pose3DToPose3DEdge(prevPose, p3dn, deltaMotion, cov);

        prevPose = p3dn;

        slam.addNode(p3dn);
        slam.addEdge(poseToPose);

        // //Turn landmarks into Point3D's
        // for(OldSimulator.landmark_t landmark : dets){
        //     double rLand = landmark.obs[0];
        //     double tLand = landmark.obs[1];

        //     double [] pos = LandUtil.rThetaToXY(rLand, tLand, x, y, t);

        //     double xLand = pos[0];
        //     double yLand = pos[1];

        //     Point3D obs = new Point3D(xLand, yLand, 0.0);

        //     Point3DNode pointNode = dataAssociation(landmark.id);

        //     if (pointNode == null) {
        //         pointNode = new Point3DNode(landmark.id);
        //         slam.addNode(pointNode);
        //     }

        //     Matrix landCov = Matrix.identity(3, 3);

        //     Pose3DToPoint3DEdge poseToPoint = new Pose3DToPoint3DEdge(p3dn, pointNode, obs, landCov);


        //     slam.addEdge(poseToPoint);
        // }

        //-------------------
        // CHOOSE YOUR WEAPON
        //-------------------
        // slam.solve();
        slam.update();

        if (numUpdates == 383) {
            long endTime = System.nanoTime();
            double elapsedTime = (endTime-startTime)/1000000000f;
            System.out.printf("Total Simulation Time: %.4f\n", elapsedTime);
        }

        drawSetup();
        drawScene(dets);
    }

    private Point3DNode dataAssociation(int idToLookFor) {

        List<Node> allNodes = slam.getNodes();

        for (Node aNode : allNodes) {

            if (aNode instanceof Point3DNode) {
                if (((Point3DNode)aNode).getId() == idToLookFor) {
                    return (Point3DNode)aNode;
                }
            }

        }

        return null;

    }

    private void drawSetup() {

        trajectory.clear();
        landmarks.clear();

        List<Node> allNodes = slam.getNodes();
        List<Edge> allEdges = slam.getEdges();

        Pose3DNode lastPose = null;

        // Get all poses and landmarks
        for (Node aNode : allNodes) {

            if (aNode instanceof Pose3DNode) {

                trajectory.add(Quiver.getQuiverAt(aNode.getStateArray(), 0.25));

                lastPose = (Pose3DNode)aNode;

            } else if (aNode instanceof Point3DNode) {

                double[] landPos = aNode.getStateArray();

                // VzPoints posGuess = new VzPoints(new VisVertexData(landPos),
                //                                  new VisConstantColor(Color.cyan),
                //                                  10.0);
                // landmarks.add(posGuess);
                landmarks.add(landPos);

            } else {

                System.out.println("Goodness! What kind of node do we have here?");

            }

        }

        // Make the last pose have a larger scale
        trajectory.remove(trajectory.size()-1);
        trajectory.add(Quiver.getQuiverAt(lastPose.getStateArray(), 1.0));


        allEdgeLinks.clear();

        // Get all links between nodes
        for (Edge anEdge : allEdges) {

            List<Node> theNodes = anEdge.getNodes();

            if (theNodes.size() == 2) {

                allEdgeLinks.add(LinAlg.resize(theNodes.get(0).getStateArray(), 3));
                allEdgeLinks.add(LinAlg.resize(theNodes.get(1).getStateArray(), 3));

            }

        }


    }

    public void drawScene(ArrayList<OldSimulator.landmark_t> landmarkObs) {

        // Draw trajectory -- the red robot path -- our least squares "best guess"
        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-local");

            for (VzLines oneQuiver : trajectory) {

                vb.addBack(oneQuiver);

            }

            vb.swap();
        }


        // Draw our least squares best guess of the landmark positions
        // {
        //     VisWorld.Buffer vb = vw.getBuffer("landmarks-local");

        //     for (VzPoints aPoint : landmarks) {

        //         vb.addBack(aPoint);

        //     }

        //     vb.swap();

        // }


        // Get the vertext data for a star
        ArrayList<double[]> star = new ArrayList<double[]>();
        int n = 5;
        double radskip = 2*(2*Math.PI/n);


        for (int i = 0; i < 2*n; i++) {
            double rad = i*radskip;
            double pt[] = {Math.cos(rad),Math.sin(rad)};
            star.add(pt);
        }

        VisVertexData vdat = new VisVertexData(star);


        // Draw the landmarks
        {

            VisWorld.Buffer vb = vw.getBuffer("landmarks-local");

            for (double[] l : landmarks) {
                vb.addBack(new VisChain(LinAlg.translate(l[0],l[1],l[2]),
                                        LinAlg.scale(.25,.25,.25),
                                        new VzLines(vdat, new VisConstantColor(Color.cyan),
                                                    2, VzLines.TYPE.LINE_LOOP)));
            }
            vb.swap();
        }


        // Draw edge links
        {
            VisWorld.Buffer vb = vw.getBuffer("edges-local");



            vb.addBack(new VzLines(new VisVertexData(allEdgeLinks),
                                   new VisConstantColor(new Color(1.0f, 1.0f, 0.0f, 0.2f)),
                                   1.0,
                                   VzLines.TYPE.LINES));

            vb.swap();
        }




        // Scene grid
        {
            VzGrid vg = new VzGrid(new Color(.5f,.5f,.5f,.2f),
                                   new Color(1.0f,1.0f,1.0f,0.0f));

            VisWorld.Buffer vb = vw.getBuffer("grid");
            vb.setDrawOrder(-100);
            vb.addBack(new VisDepthTest(false,vg));
            vb.swap();
        }



    }

}
