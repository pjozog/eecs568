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
import perllcm.ardrone_state_t;

public class DroneState implements LCMSubscriber {
    
    LCM lcm = new LCM();

    ardrone_state_t oldState = null;

    private static double ALT_FUDGE = 0.3;

    public DroneState() throws IOException {

        this.lcm.subscribe("ARDRONE_STATE", this);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
        
        try {

            if (channel.equals("ARDRONE_STATE")) {

                ardrone_state_t msg = new ardrone_state_t(ins);

                msg.altitude += ALT_FUDGE;

                double time = msg.utime;
                double velx = msg.vx;
                double vely = msg.vy;
                double velz = msg.vz;
                double alt = msg.altitude;
                double r = msg.roll;
                double p = msg.pitch;
                double y = msg.yaw;

                if (velx == 0.0 && vely == 0.0)
                    return;

                if(oldState == null){
                    oldState = msg;
                    return;
                }
                
                double delx = velx * (time - oldState.utime)/1e6;
                double dely = vely * (time - oldState.utime)/1e6;
                double delz = oldState.altitude - alt;
                if (Math.abs(delz) > 1)
                    delz = 0;
                double delr = r - oldState.roll;
                double delp = p - oldState.pitch;
                double delh = y - oldState.yaw;
                    
                pose3d_t outMsg = new pose3d_t();                    
                outMsg.utime    = System.nanoTime();
                outMsg.Sigma    = Matrix.identity(6,6).times(.01).copyAsVector();        
                
                double []deltaPose = new double[]{delx, dely, delz, delr, delp, delh};
                outMsg.mu = deltaPose;
                

                lcm.publish("ARDRONE_DELTA_POSE", outMsg);

                oldState = msg;

                return;

            }
            

        }
        catch (IOException ex) {
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
