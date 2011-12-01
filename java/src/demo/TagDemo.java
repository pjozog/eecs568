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

    double tagsize;
    double fx;
    double fy;

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

        this.config = _config;

        this.is = is;
        this.tf = tf;

        this.tagsize = this.config.requireDouble("target.width");
        this.fx = this.config.requireDouble("camera.fx");
        this.fy = this.config.requireDouble("camera.fy");

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

            VisWorld.Buffer poseDisplay = vw.getBuffer("Pose Display");

            vbOrigin.addBack(Quiver.getQuiverAt(new double[]{0,0,0,0,0,0}, 1, 
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

                if (detections.size() == 0)
                    vb.clear();

                for (TagDetection d : detections) {

                    double[] poseTagToHzCam = TagUtil.getPose(d.homography, tagsize, fx, fy);
                    Matrix Sigma = TagUtil.getPoseSigma(d.homography, d.covariance, tagsize, fx, fy);

                    poseDisplay.addBack(new VisPixelCoordinates(VisPixelCoordinates.ORIGIN.BOTTOM_RIGHT,
                                                                new VzText(VzText.ANCHOR.BOTTOM_RIGHT,
                                                                           String.format("<<cyan>>[%.2f %.2f %.2f %.2f %.2f %.2f]", 
                                                                                         poseTagToHzCam[0],
                                                                                         poseTagToHzCam[1],
                                                                                         poseTagToHzCam[2],
                                                                                         poseTagToHzCam[3],
                                                                                         poseTagToHzCam[4],
                                                                                         poseTagToHzCam[5]))));


                    tag_point3d_t msg = new tag_point3d_t();

                    msg.utime = System.nanoTime();
                    msg.id    = d.id;
                    msg.mu    = new double[]{poseTagToHzCam[0], poseTagToHzCam[1], poseTagToHzCam[2]};
                    msg.Sigma = new Matrix(Sigma.copyArray(0, 0, 3, 3)).copyAsVector();
                    lcm.publish("ARDRONE_CAM_TO_TAG", msg);

                    vb.addBack(Quiver.getQuiverAt(poseTagToHzCam));
                    poseDisplay.swap();

                }

                vb.swap();
                
            }

        }

    }

}
