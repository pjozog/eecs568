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

import bot_core.image_t;
import perllcm.pose3d_t;

public class DroneState implements LCMSubscriber {
    
    LCM lcm = new LCM();

    public DroneState() throws IOException {

        this.lcm.subscribe("ARDRONE_STATE", this);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
        
        try {

            if (channel.equals("ARDRONE_STATE")) {

                ardrone_state_t msg = new ardrone_state_t(ins);


		    double time = msg.utime;
		    double velx = msg.vx;
		    double vely = msg.vy;
		    double velz = msg.vz;
		    double alt = msg.altitude;
		    double r = msg.roll;
		    double p = msg.pitch;
		    double y = msg.yaw;

		    double[] currentPose = 



                    pose3d_t outMsg = new pose3d_t();                    
		    outMsg.utime    = System.nanoTime();
                    outMsg.mu       = deltaPose;
                    outMsg.Sigma    = Matrix.identity(6,6).copyAsVector();
                    lcm.publish("ARDRONE_DELTA_POSE", outMsg);

                    return;

                }
            }

        } catch (IOException ex) {
            System.out.println("Caught exception: "+ex);
        }

    }

    public static void main(String[] args) {

        try {

            DroneState ds = new DroneState();

            while (true) {
                Thread.sleep(1000);
            }

        } catch (IOException ex) {
            System.out.println("Caught exception: "+ex);
        } catch (InterruptedException ex) { 
        
        }

    }

}
