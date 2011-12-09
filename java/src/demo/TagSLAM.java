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

class PointColorPair {
    double[] point;
    Color color;
    public PointColorPair(double[] p, Color c) {
        this.point = p;
        this.color = c;
    }
}

public class TagSLAM {

    Config config;
    LCM lcm;
    ImageSource is;
    TagFamily tf;
    TagDetector detector;
    JFrame jf         = new JFrame("568 Interactive Demo (TagSLAM)");
    final VisWorld vw = new VisWorld();
    VisLayer vl       = new VisLayer(vw);
    VisCanvas vc      = new VisCanvas(vl);

    VisWorld vw2 = new VisWorld();
    VisLayer vl2 = new VisLayer(vw2);
    VisCanvas vc2 = new VisCanvas(vl2);
    VisWorld.Buffer vbOriginal = vw2.getBuffer("unprocessed image");
    VisWorld.Buffer vbDetections = vw2.getBuffer("detections");

    ParameterGUI pg   = new ParameterGUI();

    double tagsize;
    double fx;
    double fy;

    BackEnd slam;

    ArrayList<VzLines> trajectory = new ArrayList<VzLines>();
    ArrayList<PointColorPair> landmarks    = new ArrayList<PointColorPair>();
    ArrayList<double[]> allEdgeLinks = new ArrayList<double[]>();

    Point3DNode landmarkNode = new Point3DNode();

    double [] NwuToNed = new double[6];
    double [] latestPoseGuess = new double[6];
    double [] lastCamPose = null;
    Pose3DNode prevPose;

    //The id of the tag we'll use for "odometry"
    public static int POSE_TAG = 0;

    //How often we'll reset
    public static int MAX_UPDATES = 1000;

    //The color map
    HashMap<Integer, Color> idToColor = new HashMap<Integer, Color>();

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

            TagSLAM tt = new TagSLAM(is, tf, new ConfigFile(args[0]));

        } catch (IOException ex) {
            System.out.println("Ex: "+ex);
        }
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
                Point3DNode castedNode = (Point3DNode)aNode;
                landmarks.add(new PointColorPair(landPos, idToColor.get(castedNode.getId())));

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
            double pt[] = {.2 * Math.cos(rad),.2 * Math.sin(rad)};
            star.add(pt);
        }

        VisVertexData vdat = new VisVertexData(star);


        // Draw the landmarks
        {

            VisWorld.Buffer vb = vw.getBuffer("landmarks-local");

            for (PointColorPair pcp : landmarks) {
                double[] point = pcp.point;
                Color color = pcp.color;
                vb.addBack(new VisChain(LinAlg.translate(point[0],point[1],point[2]),
                                        LinAlg.scale(.25,.25,.25),
                                        new VzLines(vdat, new VisConstantColor(color),
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

    public TagSLAM (ImageSource is, TagFamily tf, Config _config) {

        idToColor.put(1, Color.cyan);
        idToColor.put(2, Color.yellow);
        idToColor.put(3, Color.red);
        idToColor.put(4, Color.magenta);
        idToColor.put(5, Color.green);

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

        slam = new BackEnd(config);

        detector = new TagDetector(tf);

        jf.setSize(800, 600);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);


        jf.setLayout(new BorderLayout());
        JSplitPane jsp = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, vc, vc2);
        jsp.setDividerLocation(0.5);
        jsp.setResizeWeight(0.5);

        jf.add(jsp, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);

        vl.cameraManager.fit2D(new double[]{-2,-2}, new double[]{2,2}, true);
   
        jf.setVisible(true);
        
        detector = new TagDetector(tf);

        ImageSourceFormat ifmt = is.getCurrentFormat();
        ((DefaultCameraManager) vl.cameraManager).interfaceMode = 2.5;
        vl.backgroundColor = new Color(0,0,0);
        vl2.cameraManager.fit2D(new double[] {0,0}, new double[] { ifmt.width, ifmt.height}, true);
        ((DefaultCameraManager) vl2.cameraManager).interfaceMode = 2.0;
        

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

    public void addPrior(double[] pose) {

        Matrix cov = Matrix.identity(6,6).times(100);
        
        Pose3D p3d = new Pose3D(pose);

        Pose3DNode p3dn = new Pose3DNode();
        Pose3DEdge p3de = new Pose3DEdge(p3dn, p3d, cov);

        prevPose = p3dn;

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

            VisWorld.Buffer cameraFrame = vw.getBuffer("Camera Frame");

            int count = 0;
            while (true) {

                byte buf[] = is.getFrame();
                if (buf == null)
                    continue;

                BufferedImage im = ImageConvert.convertToImage(fmt.format, fmt.width, fmt.height, buf);

                Tic tic = new Tic();
                ArrayList<TagDetection> detections = detector.process(im, new double[] {im.getWidth()/2.0, im.getHeight()/2.0});


                vbOriginal.addBack(new VzImage(im, VzImage.FLIP));
                vbOriginal.swap();
                
                double dt = tic.toc();

                
                for(TagDetection d : detections) {

                    double p0[] = d.interpolate(-1,-1);
                    double p1[] = d.interpolate(1,-1);
                    double p2[] = d.interpolate(1,1);
                    double p3[] = d.interpolate(-1,1);

                    double ymax = Math.max(Math.max(p0[1], p1[1]), Math.max(p2[1], p3[1]));

                    vbDetections.addBack(new VisChain(LinAlg.translate(0, im.getHeight(), 0),
                                                      LinAlg.scale(1, -1, 1),
                                                      new VzLines(new VisVertexData(p0, p1, p2, p3, p0),
                                                                  new VisConstantColor(Color.blue),4, VzLines.TYPE.LINE_STRIP),
                                                      new VzLines(new VisVertexData(p0,p1),
                                                                  new VisConstantColor(Color.green),4, VzLines.TYPE.LINE_STRIP), // x axis
                                                      new VzLines(new VisVertexData(p0, p3),
                                                                  new VisConstantColor(Color.red),4, VzLines.TYPE.LINE_STRIP), // y axis
                                                      new VisChain(LinAlg.translate(d.cxy[0], ymax + 20, 0), //LinAlg.translate(d.cxy[0],d.cxy[1],0),
                                                                   LinAlg.scale(1, -1, 1),
                                                                   LinAlg.scale(.25, .25, .25),
                                                                   new VzText(VzText.ANCHOR.CENTER,
                                                                              String.format("<<sansserif-48,center,red,dropshadow=#88000000>>ID=%3d\nHamming=%d\n", d.id, d.hammingDistance)))));


                    //Create a prior at the current pose (NOT the origin)
                    if (lastCamPose == null && d.id == POSE_TAG) {

                        lastCamPose = TagUtil.getPose(d.homography, tagsize, fx, fy);
                        addPrior(lastCamPose);
                        continue;

                    }

                    double[] currentCamPose = TagUtil.getPose(d.homography, tagsize, fx, fy);

                    if (d.id == POSE_TAG) {

                        //Create the pose odometry (previous camera to current camera)
                        double [] prevCamToCurrCam = SixDofCoords.tailToTail(lastCamPose, currentCamPose);
                        Matrix cov = Matrix.identity(6,6).times(1);
                        Pose3DNode newPose = new Pose3DNode();

                        Pose3DToPose3DEdge poseToPose = new Pose3DToPose3DEdge(prevPose, newPose, new Pose3D(prevCamToCurrCam), cov);
                        lastCamPose = currentCamPose;
                        prevPose = newPose;

                        slam.addNode(newPose);
                        slam.addEdge(poseToPose);


                    } else {

                        if (prevPose == null) {
                            continue;
                        }

                        //Create the landmark observation (remember: from camera to target)
                        double[] camToTag = SixDofCoords.inverse(currentCamPose);
                        double[] landmarkObs = new double[]{camToTag[0], camToTag[1], camToTag[2]};
                        Matrix obsCov = Matrix.identity(3,3);

                        Point3D obs = new Point3D(landmarkObs);
                        Point3DNode pointNode = dataAssociation(d.id);

                        if (pointNode == null) {
                            pointNode = new Point3DNode(d.id);
                            slam.addNode(pointNode);
                        }

                        Pose3DToPoint3DEdge poseToPoint = new Pose3DToPoint3DEdge(prevPose, pointNode, obs, obsCov);
                        slam.addEdge(poseToPoint);

                    }

                    count++;
                    slam.update();

                    drawSetup();
                    drawScene();

                    if (count % MAX_UPDATES == 0) {
                        reset();
                    }

                }

                vbDetections.swap();

            }

        }

    }

    public void reset() {

        trajectory.clear();
        landmarks.clear();
        allEdgeLinks.clear();

        vw.getBuffer("edges-local").clear();
        vw.getBuffer("trajectory-local").clear();
        vw.getBuffer("landmarks-local").clear();

        slam = new BackEnd(this.config);

        lastCamPose = null;
        prevPose = null;

    }

}
