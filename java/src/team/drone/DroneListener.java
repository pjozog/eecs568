package team.drone;

import team.common.*;
import team.slam.*;

import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.io.*;

import april.vis.*;
import april.config.*;
import april.jmat.*;
import april.util.*;

import lcm.lcm.*;
import perllcm.pose3d_t;
import perllcm.tag_point3d_t;

public class DroneListener implements LCMSubscriber, ParameterListener {

    Config config;
    LCM lcm;
    JFrame jf    = new JFrame("Drone Listener Process");
    VisWorld  vw = new VisWorld();
    VisLayer vl  = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);
    VzGrid vg    = new VzGrid(new Color(.5f, .5f, .5f),
                              new Color(1.0f, 1.0f, 1.0f, 0.0f));

    ParameterGUI pg = new ParameterGUI();

    double[] currentPose = new double[6];
    double[] robotToCam  = new double[6];
    double[] NwuToNed    = new double[6];

    double[] latestPoseGuess = new double[6];

    BackEnd slam;
    Pose3DNode prevPose; //Pointer to most recently created pose3d node

    ArrayList<double[]> poses        = new ArrayList<double[]>();
    ArrayList<VzLines> trajectory    = new ArrayList<VzLines>();
    ArrayList<double[]> landmarks    = new ArrayList<double[]>();
    ArrayList<double[]> allEdgeLinks = new ArrayList<double[]>();

    public DroneListener(Config config) throws IOException {

        pg.addButtons("reset", "Reset");
        pg.addListener(new ParameterListener() {
                public void parameterChanged(ParameterGUI pg, String name) {
                    if (name.equals("reset")) {
                        poseReset();
                    }
                }
            });

        try {
            lcm = new LCM();
        } catch (IOException ex) {
            System.out.println("Exception: "+ex);
        }

        this.config = config;

        robotToCam = config.requireDoubles("robot.vehicle_to_camera");
        NwuToNed = config.requireDoubles("robot.NwuToNed");

        poseReset();

        jf.setLayout(new BorderLayout());
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);
        jf.setVisible(true);
        jf.setSize(800,600);

        VisWorld.Buffer vb = vw.getBuffer("grid");
        vb.setDrawOrder(-100);
        vb.addBack(new VisDepthTest(false,vg));
        vb.swap();
        vl.cameraManager.fit2D(new double[]{-2,-2}, new double[]{2,2}, true);

        this.lcm.subscribe("ARDRONE_CAM_TO_TAG", this);
        this.lcm.subscribe("ARDRONE_DELTA_POSE", this);

        slam = new BackEnd(config);
        addPrior();

    }

    public void poseReset() {

        poses.clear();
        trajectory.clear();
        landmarks.clear();
        allEdgeLinks.clear();

        vw.getBuffer("edges-local").clear();
        vw.getBuffer("trajectory-local").clear();
        vw.getBuffer("landmarks-local").clear();

        slam = new BackEnd(this.config);

        poses.add(new double[]{currentPose[0], currentPose[1], currentPose[2]});
        addPrior();

    }

    public void parameterChanged(ParameterGUI pg, String name) {
        
    }

    public void addPrior() {

        Matrix cov = Matrix.identity(6, 6);
        cov.times(.0001);

        // Create Pose3D at origin
        Pose3D p3d = new Pose3D(NwuToNed);

        // Create Pose3DEdge
        Pose3DNode p3dn = new Pose3DNode();
        Pose3DEdge p3de = new Pose3DEdge(p3dn, p3d, cov);

        prevPose = p3dn;

        slam.addNode(p3dn);
        slam.addEdge(p3de);

    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
        
        if (jf == null)
            return;

        try {
            
            if (channel.equals("ARDRONE_DELTA_POSE")) {

                pose3d_t msg = new pose3d_t(ins);
                double[] relPose = msg.mu;

                Pose3D deltaMotion = new Pose3D(relPose);
                Pose3DNode p3dn = new Pose3DNode();
                Matrix cov = Matrix.columnPackedMatrix(msg.Sigma, 6, 6);
                //Matrix cov = Matrix.identity(6, 6);
                
                Pose3DToPose3DEdge poseToPose = new Pose3DToPose3DEdge(prevPose, p3dn, deltaMotion, cov);

                prevPose = p3dn;

                slam.addNode(p3dn);
                slam.addEdge(poseToPose);

            } else if (channel.equals("ARDRONE_CAM_TO_TAG")) {

                //We gotta convert the point in camera frame to the
                //robot's frame
                tag_point3d_t msg = new tag_point3d_t(ins);
                double[] camToPoint = msg.mu;
                double[] _robotToPoint = SixDofCoords.headToTail(robotToCam, new double[]{camToPoint[0], camToPoint[1], camToPoint[2], 0.0, 0.0, 0.0});
                double[] robotToPoint = new double[]{_robotToPoint[0], _robotToPoint[1], _robotToPoint[2]};

                //For drawing
                double[] _worldToPoint = SixDofCoords.headToTail(currentPose, _robotToPoint);
                double[] worldToPoint = new double[]{_worldToPoint[0], _worldToPoint[1], _worldToPoint[2]};

                Point3D obs = new Point3D(robotToPoint);
                Matrix cov = Matrix.columnPackedMatrix(msg.Sigma, 3, 3);
                java.util.List<Point3DNode> pointNodes = dataAssociation(msg.id);

                if (pointNodes.size() == 0) {

                    Point3DNode pointNode = new Point3DNode(msg.id);
                    Pose3DToPoint3DEdge poseToPoint = new Pose3DToPoint3DEdge(prevPose, pointNode, obs, cov);
                    slam.addEdge(poseToPoint);
                    slam.addNode(pointNode);

                } else {
                    
                    for (Point3DNode n : pointNodes) {

                        Pose3DToPoint3DEdge poseToPoint = new Pose3DToPoint3DEdge(prevPose, n, obs, cov);
                        slam.addEdge(poseToPoint);

                    }


                }



            }

        } catch (IOException ex) {
            System.out.println("Caught exception: "+ex);
        }

        slam.update();

        drawSetup();
        drawScene();

    }

    private void drawSetup() {

        trajectory.clear();
        landmarks.clear();

        java.util.List<Node> allNodes = slam.getNodes();
        java.util.List<Edge> allEdges = slam.getEdges();

        Pose3DNode lastPose = null;

        // Get all poses and landmarks
        for (Node aNode : allNodes) {

            if (aNode instanceof Pose3DNode) {

                trajectory.add(Quiver.getQuiverAt(aNode.getStateArray(), 0.01));

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
        trajectory.add(Quiver.getQuiverAt(lastPose.getStateArray(), .05));

        latestPoseGuess = lastPose.getStateArray();


        allEdgeLinks.clear();

        // Get all links between nodes
        for (Edge anEdge : allEdges) {

            java.util.List<Node> theNodes = anEdge.getNodes();

            if (theNodes.size() == 2) {

                allEdgeLinks.add(LinAlg.resize(theNodes.get(0).getStateArray(), 3));
                allEdgeLinks.add(LinAlg.resize(theNodes.get(1).getStateArray(), 3));

            }

        }


    }

    public void drawScene() {

        // Draw trajectory -- the red robot path -- our least squares "best guess"
        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-local");

            for (VzLines oneQuiver : trajectory) {

                vb.addBack(oneQuiver);

            }

            vb.swap();
        }


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

    private java.util.List<Point3DNode> dataAssociation(int idToLookFor) {

        java.util.List<Point3DNode> nodeList = new ArrayList<Point3DNode>();
        java.util.List<Node> allNodes = slam.getNodes();

        for (Node aNode : allNodes) {

            if (aNode instanceof Point3DNode) {
                if (((Point3DNode)aNode).getId() == idToLookFor) {
                    nodeList.add((Point3DNode)aNode);
                }
            }

        }

        return nodeList;

    }

    public static void main(String[] args) {

        if (args.length != 1) {
            System.out.println("You require a config file");
            return;
        }
        
        try {

            DroneListener dt = new DroneListener(new ConfigFile(args[0]));

            while (true) {
                Thread.sleep(100);
            }

        } catch (IOException ex) {
            System.out.println("Caught exception: "+ex);
        } catch (InterruptedException ex) { 
        
        }

    }

    
}
