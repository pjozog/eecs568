package team.drone;

import java.io.*;
import java.util.*;
import java.awt.image.*;
import javax.swing.*;
import java.awt.*;

import lcm.lcm.*;

import april.jcam.ImageConvert;
import april.tag.*;
import april.vis.*;
import april.jmat.*;
import april.config.*;

import team.common.SixDofCoords;
import team.common.TagUtil;

import bot_core.image_t;
import perllcm.tag_pose3d_t;

public class DroneTag implements LCMSubscriber {
    
    Config config;
    LCM lcm                      = new LCM();
    TagFamily tf                 = new Tag36h11();
    TagDetector detector         = new TagDetector(tf);
    JFrame jf                    = new JFrame("Drone Tag Process");
    VisWorld  vw                 = new VisWorld();
    VisLayer vl                  = new VisLayer(vw);
    VisCanvas vc                 = new VisCanvas(vl);
    VisWorld.Buffer vbOriginal   = vw.getBuffer("unprocessed image");
    VisWorld.Buffer vbDetections = vw.getBuffer("detections");

    public DroneTag(Config config) throws IOException {

        this.config = config;

        jf.setLayout(new BorderLayout());
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.add(vc, BorderLayout.CENTER);
        jf.setVisible(true);

        this.lcm.subscribe("ARDRONE_CAM", this);

    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {

        if (jf == null)
            return;
        
        try {

            if (channel.equals("ARDRONE_CAM")) {

                image_t msg = new image_t(ins);

                byte[] imageData = msg.data;

                BufferedImage im = ImageConvert.convertToImage("RGB",
                                                               msg.width,
                                                               msg.height,
                                                               imageData);

                vl.cameraManager.fit2D(new double[] {0,0}, new double[] { msg.width, msg.height}, true);
                ((DefaultCameraManager) vl.cameraManager).interfaceMode = 2.0;
    
                vbOriginal.addBack(new VzImage(im, VzImage.FLIP));
                vbOriginal.swap();

                ArrayList<TagDetection> detections = detector.process(im, new double[] {msg.width/2.0, msg.height/2.0});

                for (TagDetection d : detections) {
                    
                    double p0[] = d.interpolate(-1,-1);
                    double p1[] = d.interpolate(1,-1);
                    double p2[] = d.interpolate(1,1);
                    double p3[] = d.interpolate(-1,1);

                    double ymax = Math.max(Math.max(p0[1], p1[1]), Math.max(p2[1], p3[1]));

                    vbDetections.addBack(new VisChain(LinAlg.translate(0, msg.height, 0),
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

                    vbDetections.swap();

                    double tagsize = config.requireDouble("target.width");
                    double fx = config.requireDouble("camera.fx");
                    double fy = config.requireDouble("camera.fy");

                    double[] poseTagToCam = TagUtil.getPose(d.homography, tagsize, fx, fy);
                    Matrix poseSigma = TagUtil.getPoseSigma(d.homography, d.covariance, tagsize, fx, fy);
                    double[] poseCamToTag = SixDofCoords.inverse(poseTagToCam);

                    tag_pose3d_t outMsg = new tag_pose3d_t();
                    outMsg.utime    = System.nanoTime();
                    outMsg.id       = d.id;
                    outMsg.mu       = poseCamToTag;
                    outMsg.Sigma    = poseSigma.copyAsVector();
                    lcm.publish("ARDRONE_CAM_TO_TAG", outMsg);

                    return;

                }

                vbDetections.clear();

            }

        } catch (IOException ex) {
            System.out.println("Caught exception: "+ex);
        }

    }

    public static void main(String[] args) {

        if (args.length != 1) {
            System.out.println("You requrie a config file, dammit");
            return;
        }
        
        try {

            DroneTag dt = new DroneTag(new ConfigFile(args[0]));

            while (true) {
                Thread.sleep(1000);
            }

        } catch (IOException ex) {
            System.out.println("Caught exception: "+ex);
        } catch (InterruptedException ex) { 
        
        }

    }

}
