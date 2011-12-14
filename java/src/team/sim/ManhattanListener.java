package team.sim;

import team.common.*;
import team.slam.*;

import javax.swing.*;
import java.awt.Color;
import java.awt.BorderLayout;
import java.util.*;
import java.io.*;

import april.vis.*;
import april.config.*;
import april.jmat.*;
import april.util.*;

public class ManhattanListener {

    // private VisWorld vw;
    // private Config config;
    // private BackEnd slam;

    // private double baseline;
    // private double lambda;
    // private double epsilon;

    // private int numConverge;

    // Pose3DNode prevPose;

    // // Profiling
    // private long startTime;
    // private int numUpdates = 0;
    private boolean saveChi2 = false;


    private int numNodes = 0;

    // // Drawing
    // ArrayList<VzLines> trajectory = new ArrayList<VzLines>();
    // // ArrayList<VzPoints> landmarks = new ArrayList<VzPoints>();
    // ArrayList<double[]> landmarks = new ArrayList<double[]>();
    // ArrayList<double[]> allEdgeLinks = new ArrayList<double[]>();
    // double[] latestXYTGuess = new double[3];


    Config config;

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



    public static void main(String[] args) {

        if (args.length != 1) {
            System.out.println("You require a config file");
            return;
        }

        try {

            ManhattanListener dt = new ManhattanListener(new ConfigFile(args[0]));


            // Read from file, add to slam, draw stuff

            String pathToData = "/Users/patokeefe/Projects/Graduate/568-ps1-ps2/java/src/team/sim/manhattanOlson3500.txt";


            FileReader fRead = null;

            try {
                fRead = new FileReader(pathToData);
            } catch (IOException e) {
                System.out.println("Can't open file!");
            }

            Scanner scan = new Scanner(fRead);

            while (scan.hasNext()) {

                String tmp = scan.next("ODOMETRY");

                int nodeIndexOne = scan.nextInt();
                int nodeIndexTwo = scan.nextInt();

                double deltaX = scan.nextDouble();
                double deltaY = scan.nextDouble();
                double deltaT = scan.nextDouble();

                // Advance past sqrtinf matrix
                for (int i = 0; i < 6; i++) {
                    double tmp2 = scan.nextDouble();
                }


                dt.newMeasurement(nodeIndexOne, nodeIndexTwo, deltaX, deltaY, deltaT);

            }


            // Print finishing stats
            System.out.println("DONE WITH SIMULATION");



            try {
                fRead.close();
            } catch (IOException e) {
                System.out.println("Can't close file!");
            }



        } catch (Exception ex) {
            System.out.println("Caught exception: "+ex);
        }
    }


    public void newMeasurement(int indexOne,
                               int indexTwo,
                               double deltaX,
                               double deltaY,
                               double deltaT) {


        // System.out.println("Adding measurement node indices "+indexOne+" "+indexTwo+" "+" totalNodes "+numNodes);


        Pose3DNode p3dnOne;
        Pose3DNode p3dnTwo;

        if (indexOne > numNodes) {
            p3dnOne = new Pose3DNode();
            slam.addNode(p3dnOne);
            numNodes++;
        } else {
            p3dnOne = (Pose3DNode)slam.getNodes().get(indexOne);
            // System.out.println("Getting old node one");
        }

        if (indexTwo > numNodes) {
            p3dnTwo = new Pose3DNode();
            slam.addNode(p3dnTwo);
            numNodes++;
        } else {
            p3dnTwo = (Pose3DNode)slam.getNodes().get(indexTwo);
            // System.out.println("Getting old node two");
        }

        Pose3D deltaMotion = new Pose3D(deltaX, deltaY, 0, 0, 0, deltaT);

        Matrix cov = Matrix.identity(6, 6);
        cov.times(2000);


        Pose3DToPose3DEdge poseToPose = new Pose3DToPose3DEdge(p3dnOne, p3dnTwo, deltaMotion, cov);

        slam.addEdge(poseToPose);

        slam.update();

        drawSetup();
        drawScene();

        // try {
        //     Thread.sleep(1000);
        // } catch (IOException ex) {
        //     System.out.println("Caught exception: "+ex);
        // } catch (InterruptedException ex) {

        // }

        // try{
        //     //do what you want to do before sleeping
        //     Thread.currentThread().sleep(1000);//sleep for 1000 ms
        //     //do what you want to do after sleeptig
        // }
        // catch(InterruptedException ie){
        //     //If this thread was intrrupted by nother thread
        // }



    }

    public ManhattanListener(Config config) {

        // pg.addButtons("reset", "Reset");
        // pg.addListener(new ParameterListener() {
        //         public void parameterChanged(ParameterGUI pg, String name) {
        //             if (name.equals("reset")) {
        //                 poseReset();
        //             }
        //         }
        //     });


        this.config = config;

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

        slam = new BackEnd(config);

        addPrior();

    }


    /**
     * Adds a Pose3DNode and a Pose3DEdge to the graph. This is needed so we don't have an
     * underconstrained systen.
     */
    public void addPrior() {

        System.out.println("Adding prior");


        Matrix cov = Matrix.identity(6, 6);
        cov.times(.0001);

        // Create Pose3D at origin
        Pose3D p3d = new Pose3D();

        // Create Pose3DEdge
        Pose3DNode p3dn = new Pose3DNode();
        Pose3DEdge p3de = new Pose3DEdge(p3dn, p3d, cov);

        prevPose = p3dn;

        slam.addNode(p3dn);
        slam.addEdge(p3de);

        // numNodes++;

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

        // Display normalized Chi^2 text
        {

            VisWorld.Buffer vb = vw.getBuffer("chi2");

            double theChi2 = slam.getNormalizedChi2();

            vb.addBack(new VisDepthTest(false,
                                        new VisPixelCoordinates(VisPixelCoordinates.ORIGIN.BOTTOM_RIGHT,
                                                                new VzText(VzText.ANCHOR.BOTTOM_RIGHT,
                                                                           "<<margin=15>>" +
                                                                           "<<center,sansserif-italic-10>>Normalized Chi2\n"+
                                                                           "<<center,sansserif-bold-10>>"+theChi2+"\n"))));


            if (saveChi2) {
                try {
                    FileWriter fstream = new FileWriter("analysis/chi2.txt", true);
                    BufferedWriter out = new BufferedWriter(fstream);
                    out.write(theChi2+"\n");
                    out.close();
                } catch (Exception e){
                    System.err.println("Errorz!: " + e.getMessage());
                }
            }


            vb.swap();


        }




    }


}
