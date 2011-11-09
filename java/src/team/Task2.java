package team;

import javax.swing.*;
import java.awt.*;

import java.util.*;
import java.io.*;

import april.vis.*;
import april.util.*;
import april.lcmtypes.*;

import lcm.lcm.*;

import team.scan.*;

public class Task2 implements LCMSubscriber, ParameterListener
{

    static LCM lcm = LCM.getSingleton();

    JFrame jf = new JFrame("Task2");

    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    ParameterGUI pg = new ParameterGUI();

    laser_t laser; // synchronize on 'this'

    ArrayList<double[]> origin = new ArrayList<double[]>();

    public Task2()
    {
	
        pg.addDoubleSlider("thresh","Thresh",0,1,.5);

        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);

        jf.setSize(800,600);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        origin.add(new double[]{0,0});

        lcm.subscribe("LIDAR_FRONT",this);
        lcm.subscribe("POSE",this);

    }

    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.equals("LIDAR_FRONT")) {
                this.laser = new laser_t(ins);

                update();
            } else if (channel.equals("POSE")) {
                // do nothing for Task 2
            }
        } catch(IOException e) {
            System.out.println("Failed to decode message on channel "+channel);
        }
    }


    public synchronized void update()
    {
        {
            //Draw the points, then the lines (for debugging)
            VisWorld.Buffer vb = vw.getBuffer("laser-points");
            vb.addBack(new VisPoints(new VisVertexData(laserToPoints(this.laser)),
                                     new VisConstantColor(Color.green),
                                     4));
	    
            vb.addBack(new VisPoints(new VisVertexData(this.origin),
                                     new VisConstantColor(Color.cyan),
                                     6));
            vb.swap();

            VisWorld.Buffer lineBuff = vw.getBuffer("fitted-lines");
            AggloLineFit fitter = new AggloLineFit(laserToPoints(this.laser), lineBuff);
            fitter.getLines();

        }

    }

    public void parameterChanged(ParameterGUI pg, String name)
    {
        if (name.equals("thresh"))
            update();
    }

    public static ArrayList<double[]> laserToPoints(laser_t laser)
    {
        ArrayList<double[]> points = new ArrayList<double[]>();
        for (int i = 0; i < laser.nranges; i++) {
            double theta = laser.rad0 + laser.radstep*i;
            points.add(new double[] { laser.ranges[i] * Math.cos(theta),
                                      laser.ranges[i] * Math.sin(theta) });
        }
        return points;
    }

    public static void main(String args[])
    {
        new Task2();
    }


}