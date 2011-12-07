package demo;

import javax.swing.*; // For JFrame
import java.awt.*; // For BorderLayout
import java.util.*;

import april.vis.*; // For VisCanvas etc
import april.util.*; // For Parameter GUI
import april.jmat.*;

import team.common.Quiver;
import team.common.ArrayUtil;
import team.common.SixDofCoords;
import team.common.TagUtil;

import team.slam.*;

import java.awt.*;
import java.awt.image.*;
import java.io.*;
import java.util.*;
import javax.swing.*;

import april.jmat.*;

import april.vis.*;
import april.jcam.*;

import april.tag.TagFamily;
import april.tag.TagDetector;
import april.tag.TagDetection;
import april.tag.CameraUtil;
import april.tag.Tag36h11;

import april.config.*;

import lcm.lcm.*;

import perllcm.tag_point3d_t;
import perllcm.pose3d_t;


public class TagDemo {

    Config config;
    LCM lcm;
    ImageSource is;
    TagFamily tf;
    TagDetector detector;
    JFrame jf         = new JFrame("VisDemo");
    final VisWorld vw = new VisWorld();
    VisLayer vl       = new VisLayer(vw);
    VisCanvas vc      = new VisCanvas(vl);

    VisWorld vw2 = new VisWorld();
    VisLayer vl2 = new VisLayer(vw2);
    VisCanvas vc2 = new VisCanvas(vl2);

    ParameterGUI pg   = new ParameterGUI();

    double tagsize;
    double fx;
    double fy;

    BackEnd slam;

    ArrayList<VzLines> trajectory = new ArrayList<VzLines>();
    ArrayList<double[]> allEdgeLinks = new ArrayList<double[]>();

    double [] NwuToNed = new double[6];
    double [] latestPoseGuess = new double[6];
    double [] lastCamPose = null;
    Pose3DNode prevPose;
    Pose3DNode cameraPose = null;


    public static void main(String args[]) {

        if (args.length != 1) {
            System.out.println("You must provide a config file");
            return;
        }

        try {
            ArrayList<String> urls = ImageSource.getCameraURLs();

            String url = null;
            if (urls.size()==1)
                url = urls.get(0);

            if (url == null) {
                System.out.printf("Cameras found:\n");
                for (String u : urls)
                    System.out.printf("  %s\n", u);
                System.out.printf("Please specify one on the command line.\n");
                return;
            }

            ImageSource is = ImageSource.make(url);

            TagFamily tf = new Tag36h11();
            if (args.length >= 2) {
                tf = (TagFamily) ReflectUtil.createObject(args[1]);
            }

            TagDemo tt = new TagDemo(is, tf, new ConfigFile(args[0]));

        } catch (IOException ex) {
            System.out.println("Ex: "+ex);
        }
    }

    private void drawSetup() {

        trajectory.clear();
        

        java.util.List<Node> allNodes = slam.getNodes();
        java.util.List<Edge> allEdges = slam.getEdges();

        Pose3DNode lastPose = null;
        int maxSize = pg.gi("numToDraw");
        int curEdgeSize = allEdges.size();
        int curNodeSize = allNodes.size();
        // Get all poses and landmarks
        if(curNodeSize > maxSize){
            curNodeSize = maxSize;
        }
        if(curEdgeSize > maxSize){
            curEdgeSize = maxSize;
        }


        for (Node aNode : allNodes.subList(allNodes.size() - curNodeSize, allNodes.size() - 1)) {

            if (aNode instanceof Pose3DNode) {

                trajectory.add(Quiver.getQuiverAt(aNode.getStateArray(), 0.01));

                lastPose = (Pose3DNode)aNode;

            }  else {

                System.out.println("Goodness! What kind of node do we have here?");

            }

        }

        // Make the last pose have a larger scale
        trajectory.remove(trajectory.size()-1);
        trajectory.add(Quiver.getQuiverAt(lastPose.getStateArray(), .05));

        latestPoseGuess = lastPose.getStateArray();


        allEdgeLinks.clear();

        // Get all links between nodes
        for (Edge anEdge : allEdges.subList(allEdges.size() - curEdgeSize, allEdges.size() - 1)) {

            java.util.List<Node> theNodes = anEdge.getNodes();

            if (theNodes.size() == 2) {

                allEdgeLinks.add(LinAlg.resize(theNodes.get(0).getStateArray(), 3));
                allEdgeLinks.add(LinAlg.resize(theNodes.get(1).getStateArray(), 3));

            }

        }
    }
    private void drawScene(){

        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-local");
            
            for (VzLines oneQuiver : trajectory) {
                
                vb.addBack(oneQuiver);
                
            }
            
            vb.swap();
            
        }

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


    public TagDemo(ImageSource is, TagFamily tf, Config _config) {

        try {
            lcm = new LCM();
        } catch (IOException ex) {
            System.out.println("Ex: "+ex);
        }

        this.config = _config;

        this.is = is;
        this.tf = tf;

        this.tagsize = this.config.requireDouble("target.width");
        this.fx = this.config.requireDouble("camera.fx");
        this.fy = this.config.requireDouble("camera.fy");
        NwuToNed = config.requireDoubles("robot.NwuToNed");

        slam = new BackEnd(config);
        //addPrior();

        cameraMode();

        detector = new TagDetector(tf);

        jf.setSize(800, 600);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);


        jf.setLayout(new BorderLayout());
        JSplitPane jsp = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, vc, vc2);
        jsp.setDividerLocation(0.5);
        jsp.setResizeWeight(0.5);

        jf.add(jsp, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);

        vl.cameraManager.uiLookAt(new double[] {0, 0, .2},
                                   new double[] {0, 0, -1},
                                   new double[] {0, 1, 0}, true);
        
        jf.setVisible(true);
        
        detector = new TagDetector(tf);

        ImageSourceFormat ifmt = is.getCurrentFormat();
        ((DefaultCameraManager) vl.cameraManager).interfaceMode = 2.5;
        vl.backgroundColor = new Color(0,0,0);
        vl2.cameraManager.fit2D(new double[] {0,0}, new double[] { ifmt.width, ifmt.height}, true);
        // vl.cameraManager.fit2D(new double[]{-1,-1}, new double[]{1,1}, true);

        // Drawing code
        {
            VzGrid vg = new VzGrid(new Color(.5f,.5f,.5f),
                                   new Color(1.0f,1.0f,1.0f,0.0f));

            VisWorld.Buffer vb = vw.getBuffer("grid");
            vb.setDrawOrder(-100);
            vb.addBack(new VisDepthTest(false,vg));
            vb.swap();
        }

        pg.addIntSlider("numToDraw", "Number to draw", 1, 500, 100);

        pg.addListener(new ParameterListener(){
                public void parameterChanged(ParameterGUI pg, String name)
                {
                    
                }
            });
        
        new RunThread().start();

    }

    public void cameraMode() {

        Matrix cov = Matrix.identity(6,6);

        
        Pose3D p3d = new Pose3D(NwuToNed);
        // Pose3D p3d = new Pose3D(location);

        Pose3DNode p3dn = new Pose3DNode();
        Pose3DEdge p3de = new Pose3DEdge(p3dn, p3d, cov);

        prevPose = p3dn;

        //lastCamPose = new double[] {0, 0, 0, 0, 0, 0};
        //        lastCamPose = location;

        cameraPose = p3dn;

        slam.addNode(p3dn);
        slam.addEdge(p3de);

    }

    public void addPrior(double[] location){
        Matrix cov = Matrix.identity(6,6);

        
        //        Pose3D p3d = new Pose3D(NwuToNed);
        Pose3D p3d = new Pose3D(location);

        Pose3DNode p3dn = new Pose3DNode();
        Pose3DEdge p3de = new Pose3DEdge(p3dn, p3d, cov);

        prevPose = p3dn;

        //lastCamPose = new double[] {0, 0, 0, 0, 0, 0};
        lastCamPose = location;

        slam.addNode(p3dn);
        slam.addEdge(p3de);


    }

    private Point3DNode dataAssociation(int idToLookFor){
        java.util.List<Node> allNodes = slam.getNodes();

        for(Node node : allNodes){
            
            //forgive me 
            if(node instanceof Point3DNode){
                if(((Point3DNode)node).getId() == idToLookFor){
                    return(Point3DNode)node;
                }
            }

        }
        return null;
    }

    class RunThread extends Thread {
        
        public void run() {
            is.start();
            ImageSourceFormat fmt = is.getCurrentFormat();

            VisWorld.Buffer vbOriginal = vw2.getBuffer("unprocessed image");

            // VisWorld.Buffer cameraFrame = vw.getBuffer("Camera Frame");

            // VisWorld.Buffer vb = vw.getBuffer("quiver");
            // VisWorld.Buffer vbOrigin = vw.getBuffer("origin");

            // VisWorld.Buffer poseDisplay = vw.getBuffer("Pose Display");

            //vbOrigin.addBack(Quiver.getQuiverAt(new double[]{0,0,0,0,0,0}, 1, 
            //                                   Color.yellow, Color.yellow, Color.yellow));
            // vbOrigin.swap();

    
            
            
            while (true) {

                byte buf[] = is.getFrame();
                if (buf == null)
                    continue;

                BufferedImage im = ImageConvert.convertToImage(fmt.format, fmt.width, fmt.height, buf);

                Tic tic = new Tic();
                ArrayList<TagDetection> detections = detector.process(im, new double[] {im.getWidth()/2.0, im.getHeight()/2.0});


                vbOriginal.addBack(new VisDepthTest(false, new VisLighting(false, new VzImage(im, VzImage.FLIP))));
                vbOriginal.swap();
                
                double dt = tic.toc();

                
                for(TagDetection d : detections){

                    if (cameraPose != null) {

                        double[] poseTagToHzCam = TagUtil.getPose(d.homography, tagsize, fx, fy);

                        // double [] deltaTagPose = SixDofCoords.tailToTail(lastCamPose, poseTagToHzCam);
                        Matrix cov = Matrix.identity(6,6);
                        Pose3DNode newPose = new Pose3DNode();

                        Pose3DToPose3DEdge poseToPose = new Pose3DToPose3DEdge(cameraPose, newPose, new Pose3D(poseTagToHzCam), cov);
                        slam.addNode(newPose);
                        slam.addEdge(poseToPose);

                        // lastCamPose = poseTagToHzCam;
                        // prevPose = newPose;

                        drawSetup();
                        drawScene();



                    } else {

                        if (lastCamPose == null) {
                            double[] poseTagToHzCam = TagUtil.getPose(d.homography, tagsize, fx, fy);
                            //double[] loc = SixDofCoords.getPosition(poseTagToHzCam);
                            addPrior(poseTagToHzCam);
                        } else {

                            double[] poseTagToHzCam = TagUtil.getPose(d.homography, tagsize, fx, fy);

                            double [] deltaTagPose = SixDofCoords.tailToTail(lastCamPose, poseTagToHzCam);
                            Matrix cov = Matrix.identity(6,6);
                            Pose3DNode newPose = new Pose3DNode();

                            Pose3DToPose3DEdge poseToPose = new Pose3DToPose3DEdge(prevPose, newPose, new Pose3D(deltaTagPose), cov);
                            slam.addNode(newPose);
                            slam.addEdge(poseToPose);

                            lastCamPose = poseTagToHzCam;
                            prevPose = newPose;

                            drawSetup();
                            drawScene();


                        }       

                    }            
                    // Matrix Sigma = TagUtil.getPoseSigma(d.homography, d.covariance, tagsize, fx, fy);

                    
                    // VisPixelCoordinates curCoords = new VisPixelCoordinates(VisPixelCoordinates.ORIGIN.BOTTOM_RIGHT,
                    //                                                         new VzText(VzText.ANCHOR.BOTTOM_RIGHT,
                    //                                                                    String.format("<<cyan>>[%.2f %.2f %.2f %.2f %.2f %.2f]", 
                    //                                                                                  poseTagToHzCam[0],
                    //                                                                                  poseTagToHzCam[1],
                    //                                                                                  poseTagToHzCam[2],
                    //                                                                                  poseTagToHzCam[3],
                    //                                                                                  poseTagToHzCam[4],
                    //                                                                                  poseTagToHzCam[5])));
                    // poseDisplay.addBack(curCoords);


                    /*
                      tag_point3d_t msg = new tag_point3d_t();
                    
                      msg.utime = System.nanoTime();
                      msg.id    = d.id;
                      msg.mu    = new double[]{poseTagToHzCam[0], poseTagToHzCam[1], poseTagToHzCam[2]};
                      msg.Sigma = new Matrix(Sigma.copyArray(0, 0, 3, 3)).copyAsVector();
                      lcm.publish("ARDRONE_CAM_TO_TAG", msg);
                    */
                   
                    // vb.addBack(Quiver.getQuiverAt(poseTagToHzCam));
                    // vb.addBack(Quiver.getQuiverAt(SixDofCoords.inverse(poseTagToHzCam)));
                    // poseDisplay.swap();
                   
                    /*
                      if(lastPose != null){
                        
                      double []relativePose = SixDofCoords.tailToTail(lastPose, poseTagToHzCam);
                      pose3d_t pose = new pose3d_t();
                      pose.mu = relativePose;
                      lcm.publish("DEMO_RELATIVE_POSE", pose);
                        
                      }
                    */     

                }

            }

        }

    }

}
