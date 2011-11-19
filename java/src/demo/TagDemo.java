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

public class TagQuiver {

    Config config;
    ImageSource is;
    TagFamily tf;
    TagDetector detector;
    JFrame jf         = new JFrame("VisDemo");
    final VisWorld vw = new VisWorld();
    VisLayer vl       = new VisLayer(vw);
    VisCanvas vc      = new VisCanvas(vl);
    ParameterGUI pg   = new ParameterGUI();

    static final double[] xOpenGlToHz = new double[]{0,0,0,Math.PI,0,0};

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

            TagQuiver tt = new TagQuiver(is, tf, new ConfigFile(args[0]));

        } catch (IOException ex) {
            System.out.println("Ex: "+ex);
        }
    }

    public TagQuiver(ImageSource is, TagFamily tf, Config _config) {

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

                BufferedImage im = ImageConvert.convertToImage(fmt.format, fmt.width, fmt.height, buf);

                Tic tic = new Tic();
                ArrayList<TagDetection> detections = detector.process(im, new double[] {im.getWidth()/2.0, im.getHeight()/2.0});
                double dt = tic.toc();

                for (TagDetection d : detections) {

                    double tagsize = config.requireDouble("target.width");
                    double fx = config.requireDouble("camera.fx");
                    double fy = config.requireDouble("camera.fy");

                    double M[][] = CameraUtil.homographyToPose(fx, fy, tagsize, d.homography);

                    //The CameraUtil gives us pose from camera to
                    //target.  Also, the camera is using the messed-up
                    //openGl-style camera frame.  So we need to get
                    //the pose from target to camera, then convert to
                    //Hartley-Zisserman camera convention (z forward,
                    //x right, y down)
                    double[] poseOpenGlCamToTag = LinAlg.matrixToXyzrpy(M);
                    double[] poseTagToOpenGlCam = SixDofCoords.inverse(LinAlg.matrixToXyzrpy(M));
                    double[] poseTagToHzCam     = SixDofCoords.headToTail(poseTagToOpenGlCam, xOpenGlToHz);

                    poseDisplay.addBack(new VisPixelCoordinates(VisPixelCoordinates.ORIGIN.BOTTOM_RIGHT,
                                                                new VzText(VzText.ANCHOR.BOTTOM_RIGHT,
                                                                           String.format("<<cyan>>[%.2f %.2f %.2f %.2f %.2f %.2f]", 
                                                                                         poseTagToHzCam[0],
                                                                                         poseTagToHzCam[1],
                                                                                         poseTagToHzCam[2],
                                                                                         poseTagToHzCam[3],
                                                                                         poseTagToHzCam[4],
                                                                                         poseTagToHzCam[5]))));

                    vb.addBack(cam.getQuiverAt(poseTagToHzCam));
                    vb.swap();
                    poseDisplay.swap();
                }

            }

        }

    }

}