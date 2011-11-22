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
    ParameterGUI pg   = new ParameterGUI();

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

    public TagDemo(ImageSource is, TagFamily tf, Config _config) {

        try {
            lcm = new LCM();
        } catch (IOException ex) {
            System.out.println("Ex: "+ex);
        }

        config = _config;

        this.is = is;
        this.tf = tf;

        detector = new TagDetector(tf);

        jf.setSize(640,480);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);


        jf.setLayout(new BorderLayout());
        jf.add(vc,BorderLayout.CENTER);
        jf.add(pg,BorderLayout.SOUTH);

        jf.setVisible(true);
        
        detector = new TagDetector(tf);

        ImageSourceFormat ifmt = is.getCurrentFormat();
        ((DefaultCameraManager) vl.cameraManager).interfaceMode = 2.5;
        vl.backgroundColor = new Color(0,0,0);

        // Drawing code
        {
            VzGrid vg = new VzGrid(new Color(.5f,.5f,.5f),
                                     new Color(1.0f,1.0f,1.0f,0.0f));

            VisWorld.Buffer vb = vw.getBuffer("grid");
            vb.setDrawOrder(-100);
            vb.addBack(new VisDepthTest(false,vg));
            vb.swap();
        }

        pg.addListener(new ParameterListener(){
                public void parameterChanged(ParameterGUI pg, String name)
                {

                }
            });
        
        new RunThread().start();

    }

    class RunThread extends Thread {
        
        public void run() {
            is.start();
            ImageSourceFormat fmt = is.getCurrentFormat();

            VisWorld.Buffer cameraFrame = vw.getBuffer("Camera Frame");

            VisWorld.Buffer vb = vw.getBuffer("quiver");
            VisWorld.Buffer vbOrigin = vw.getBuffer("origin");

            Quiver origin = new Quiver();
            Quiver cam = new Quiver();
            VisWorld.Buffer poseDisplay = vw.getBuffer("Pose Display");

            vbOrigin.addBack(origin.getQuiverAt(new double[]{0,0,0,0,0,0}, 1, 
                                                Color.yellow, Color.yellow, Color.yellow));
            vbOrigin.swap();

            while (true) {
                byte buf[] = is.getFrame();
                if (buf == null)
                    continue;

                BufferedImage im = ImageConvert.convertToImage("BE_GRAY16", fmt.width, fmt.height, buf);

                Tic tic = new Tic();
                ArrayList<TagDetection> detections = detector.process(im, new double[] {im.getWidth()/2.0, im.getHeight()/2.0});
                double dt = tic.toc();

                if (detections.size() == 0)
                    vb.clear();

                for (TagDetection d : detections) {

                    double[] poseTagToHzCam = getPose(d.homography);
                    Matrix J = new Matrix(getPoseJacob(d.homography));

                    Matrix Sigma = J.times(new Matrix(d.covariance)).times(J.transpose());

                    // System.out.println("--------------------------------------------------");
                    // ArrayUtil.print2dArray(getPoseJacob(d.homography));
                    // System.out.println();
                    // ArrayUtil.print2dArray(Sigma.copyArray());
                    // System.out.println("--------------------------------------------------");

                    poseDisplay.addBack(new VisPixelCoordinates(VisPixelCoordinates.ORIGIN.BOTTOM_RIGHT,
                                                                new VzText(VzText.ANCHOR.BOTTOM_RIGHT,
                                                                           String.format("<<cyan>>[%.2f %.2f %.2f %.2f %.2f %.2f]", 
                                                                                         poseTagToHzCam[0],
                                                                                         poseTagToHzCam[1],
                                                                                         poseTagToHzCam[2],
                                                                                         poseTagToHzCam[3],
                                                                                         poseTagToHzCam[4],
                                                                                         poseTagToHzCam[5]))));


                    pose3d_t msg = new pose3d_t();

                    msg.utime = System.nanoTime();
                    msg.mu    = poseTagToHzCam;
                    msg.Sigma = Matrix.identity(6,6).copyAsVector();
                    lcm.publish("ARDRONE_CAM_TO_TAG", msg);

                    vb.addBack(cam.getQuiverAt(poseTagToHzCam));
                    vb.swap();
                    poseDisplay.swap();
                }

            }

        }

        public double[] getPose(double[][] H) {
            
            double tagsize = config.requireDouble("target.width");
            double fx = config.requireDouble("camera.fx");
            double fy = config.requireDouble("camera.fy");

            double M[][] = CameraUtil.homographyToPose(fx, fy, tagsize, H);

            //The CameraUtil gives us pose from camera to
            //target.  Also, the camera is using the messed-up
            //openGl-style camera frame.  So we need to get
            //the pose from target to camera, then convert to
            //Hartley-Zisserman camera convention (z forward,
            //x right, y down)
            double[] poseOpenGlCamToTag = LinAlg.matrixToXyzrpy(M);
            double[] poseTagToOpenGlCam = SixDofCoords.inverse(LinAlg.matrixToXyzrpy(M));
            double[] poseTagToHzCam     = SixDofCoords.headToTail(poseTagToOpenGlCam, SixDofCoords.xOpenGlToHz);
            return poseTagToHzCam;

        }

        public double[][] getPoseJacob(double[][] H) {
            
            Matrix HMat = new Matrix(H);
            double[] h = HMat.getColumnPackedCopy();

            double eps = 1e-6;

            Matrix J = new Matrix(6, 9);

            //Differentiate with respect to h
            for (int i = 0; i < 9; i++) {
                for (int j = 0; j < 6; j++) {

                    double[] hPert = new double[h.length];
                    for (int k = 0; k < h.length; k++)
                        hPert[k] = h[k];

                    hPert[i] += eps;

                    double[] y = getPose(hVecToMat(h));
                    double[] yPert = getPose(hVecToMat(hPert));

                    double finiteDiff = (yPert[j] - y[j]) / eps;

                    J.set(j, i, finiteDiff);
                
                }
            }
            
            return J.copyArray();
            
        }

        public double[][] hVecToMat(double[] h) {
            assert(h.length == 9);

            double[][] H = new double[3][3];

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    H[i][j] = h[i*3+j];

            return H;
        }

    }

}
