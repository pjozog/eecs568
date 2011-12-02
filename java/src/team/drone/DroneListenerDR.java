package team.drone;

import team.common.*;

import javax.swing.*;
import java.awt.*;
import java.util.*;
import java.io.*;

import april.vis.*;
import april.config.*;
import april.jmat.*;

import lcm.lcm.*;
import perllcm.pose3d_t;
import perllcm.tag_point3d_t;

//Dead reckoning
public class DroneListenerDR implements LCMSubscriber {

    Config config;
    LCM lcm;
    JFrame jf    = new JFrame("Drone Tag Process");
    VisWorld  vw = new VisWorld();
    VisLayer vl  = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);
    VzGrid vg    = new VzGrid(new Color(.5f, .5f, .5f),
                              new Color(1.0f, 1.0f, 1.0f, 0.0f));

    double[] currentPose = new double[6];
    double[] robotToCam = new double[6];
    double[] NwuToNed = new double[6];

    ArrayList<double[]> poses = new ArrayList<double[]>();

    //For dead reckoning
    VisWorld.Buffer vbPose; 
    VisWorld.Buffer vbLandmark; 
    VisWorld.Buffer vbTraj; 

    public DroneListenerDR(Config config) throws IOException {

        try {
            lcm = new LCM();
        } catch (IOException ex) {
            System.out.println("Exception: "+ex);
        }

        this.config = config;

        robotToCam = config.requireDoubles("robot.vehicle_to_camera");
        NwuToNed = config.requireDoubles("robot.NwuToNed");
        currentPose = SixDofCoords.headToTail(currentPose, NwuToNed);
        poses.add(new double[]{currentPose[0], currentPose[1], currentPose[2]});

        jf.setLayout(new BorderLayout());
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.add(vc, BorderLayout.CENTER);
        jf.setVisible(true);
        jf.setSize(800,600);

        VisWorld.Buffer vb = vw.getBuffer("grid");
        vb.setDrawOrder(-100);
        vb.addBack(new VisDepthTest(false,vg));
        vb.swap();

        vbPose = vw.getBuffer("Pose Display");
        vbLandmark = vw.getBuffer("Landmark Display");
        vbTraj = vw.getBuffer("Trajectory");

        this.lcm.subscribe("ARDRONE_CAM_TO_TAG", this);
        this.lcm.subscribe("ARDRONE_DELTA_POSE", this);

    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
        
        if (jf == null)
            return;

        try {
            
            if (channel.equals("ARDRONE_DELTA_POSE")) {

                pose3d_t msg = new pose3d_t(ins);
                double[] relPose = msg.mu;
                currentPose = SixDofCoords.headToTail(currentPose, relPose);
                poses.add(new double[]{currentPose[0], currentPose[1], currentPose[2]});

                vbPose.addBack(Quiver.getQuiverAt(currentPose));
                vbPose.swap();

                vbTraj.addBack(new VzLines(new VisVertexData(poses),
                                           new VisConstantColor(Color.red), 2, VzLines.TYPE.LINES));
                vbTraj.swap();
                    
            } else if (channel.equals("ARDRONE_CAM_TO_TAG")) {

                tag_point3d_t msg = new tag_point3d_t(ins);
                double[] camToPoint = msg.mu;
                double[] _robotToPoint = SixDofCoords.headToTail(robotToCam, new double[]{camToPoint[0], camToPoint[1], camToPoint[2], 0.0, 0.0, 0.0});
                double[] _worldToPoint = SixDofCoords.headToTail(currentPose, _robotToPoint);
                double[] worldToPoint = new double[]{_worldToPoint[0], _worldToPoint[1], _worldToPoint[2]};

                ArrayList<double[]> points = new ArrayList<double[]>();
                points.add(new double[]{currentPose[0], currentPose[1], currentPose[2]});
                points.add(worldToPoint);
                
                vbLandmark.addBack(new VzLines(new VisVertexData(points),
                                               new VisConstantColor(Color.cyan), 2, VzLines.TYPE.LINES));
                vbLandmark.swap();


            }

        } catch (IOException ex) {
            System.out.println("Caught exception: "+ex);
        }

    }

    public static void main(String[] args) {

        if (args.length != 1) {
            System.out.println("You require a config file");
            return;
        }
        
        try {

            DroneListenerDR dt = new DroneListenerDR(new ConfigFile(args[0]));

            while (true) {
                Thread.sleep(100);
                dt.vbLandmark.clear();
            }

        } catch (IOException ex) {
            System.out.println("Caught exception: "+ex);
        } catch (InterruptedException ex) { 
        
        }

    }

    
}
