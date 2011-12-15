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


    private boolean saveChi2 = false;
    private int numNodes = 0;


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


    BackEnd slam;
    Pose2DNode prevPose; //Pointer to most recently created pose3d node

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

            //String pathToData = "/Users/patokeefe/Projects/Graduate/568-ps1-ps2/java/src/team/sim/manhattanOlson3500.txt";
            String pathToData = "./src/team/sim/manhattanOlson3500.txt";

            FileReader fRead = null;

            try {
                fRead = new FileReader(pathToData);
            } catch (IOException e) {
                System.out.println("Can't open file!");
            }

            Scanner scan = new Scanner(fRead);

            long startTime = System.nanoTime();

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

            long endTime = System.nanoTime();
            double elapsedTime = (endTime-startTime)/1000000000f;
            // Print finishing stats
            System.out.println("DONE WITH SIMULATION");
            System.out.printf("\nTotal Simulation Time: %.4f\n", elapsedTime);


            try {
                fRead.close();
            } catch (IOException e) {
                System.out.println("Can't close file!");
            }



        } catch (Exception ex) {
            System.out.println("Caught exception: "+ex);
            ex.printStackTrace();
        }
    }


    public void newMeasurement(int indexOne,
                               int indexTwo,
                               double deltaX,
                               double deltaY,
                               double deltaT) {


        // System.out.println("Adding measurement node indices "+indexOne+" "+indexTwo+" "+" totalNodes "+numNodes);


        Pose2DNode p2dnOne;
        Pose2DNode p2dnTwo;

        if (indexOne > numNodes) {
            p2dnOne = new Pose2DNode();
            slam.addNode(p2dnOne);
            numNodes++;
        } else {
            p2dnOne = (Pose2DNode)slam.getNodes().get(indexOne);
            // System.out.println("Getting old node one");
        }

        if (indexTwo > numNodes) {
            p2dnTwo = new Pose2DNode();
            slam.addNode(p2dnTwo);
            numNodes++;
        } else {
            p2dnTwo = (Pose2DNode)slam.getNodes().get(indexTwo);
            // System.out.println("Getting old node two");
        }

        Pose2D deltaMotion = new Pose2D(deltaX, deltaY, deltaT);

        Matrix cov = Matrix.identity(3, 3);
        cov = cov.times(1.0/2000.0);



        Pose2DToPose2DEdge poseToPose = new Pose2DToPose2DEdge(p2dnOne, p2dnTwo, deltaMotion, cov);

        slam.addEdge(poseToPose);

        slam.update();

        double theChi2 = slam.getNormalizedChi2();
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


        // drawSetup();
        // drawScene();

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
        vl.cameraManager.fit2D(new double[]{-20,-20}, new double[]{20,20}, true);

        slam = new BackEnd(config);

        addPrior();

    }


    /**
     * Adds a Pose2DNode and a Pose2DEdge to the graph. This is needed so we don't have an
     * underconstrained systen.
     */
    public void addPrior() {

        System.out.println("Adding prior");


        Matrix cov = Matrix.identity(3, 3);
        cov = cov.times(1.0/2000);

        // Create Pose2D at origin
        Pose2D p2d = new Pose2D();

        // Create Pose2DEdge
        Pose2DNode p2dn = new Pose2DNode();
        Pose2DEdge p2de = new Pose2DEdge(p2dn, p2d, cov);

        prevPose = p2dn;

        slam.addNode(p2dn);
        slam.addEdge(p2de);


    }


    private void drawSetup() {

        trajectory.clear();
        landmarks.clear();

        java.util.List<Node> allNodes = slam.getNodes();
        java.util.List<Edge> allEdges = slam.getEdges();

        Pose2DNode lastPose = null;

        // Get all poses and landmarks
        for (Node aNode : allNodes) {

            if (aNode instanceof Pose2DNode) {

                double[] realPose = aNode.getStateArray();

                double[] fakePose = new double[6];
                fakePose[0] = realPose[0];
                fakePose[1] = realPose[1];
                fakePose[2] = 0.0;
                fakePose[3] = 0.0;
                fakePose[4] = 0.0;
                fakePose[5] = realPose[2];

                trajectory.add(Quiver.getQuiverAt(fakePose, 0.01));

                lastPose = (Pose2DNode)aNode;

            } else if (aNode instanceof Point2DNode) {

                double[] landPos = aNode.getStateArray();

                landmarks.add(landPos);

            } else {

                System.out.println("Goodness! What kind of node do we have here?");

            }

        }


        double[] realPose = lastPose.getStateArray();

        double[] fakePose = new double[6];
        fakePose[0] = realPose[0];
        fakePose[1] = realPose[1];
        fakePose[2] = 0.0;
        fakePose[3] = 0.0;
        fakePose[4] = 0.0;
        fakePose[5] = realPose[2];

        // Make the last pose have a larger scale
        trajectory.remove(trajectory.size()-1);
        trajectory.add(Quiver.getQuiverAt(fakePose, .05));


        allEdgeLinks.clear();

        // Get all links between nodes
        for (Edge anEdge : allEdges) {

            java.util.List<Node> theNodes = anEdge.getNodes();

            if (theNodes.size() == 2) {

                allEdgeLinks.add(LinAlg.resize(theNodes.get(0).getStateArray(), 2));
                allEdgeLinks.add(LinAlg.resize(theNodes.get(1).getStateArray(), 2));

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
