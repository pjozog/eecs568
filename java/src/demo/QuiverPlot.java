package demo;

import javax.swing.*; // For JFrame
import java.awt.*; // For BorderLayout
import java.util.*;

import april.vis.*; // For VisCanvas etc
import april.util.*; // For Parameter GUI
import april.jmat.*;

import team.common.Quiver;

public class QuiverPlot {
    
    public static double[] relPose;
    public static double frameSize;

    public static void main(String[] args) {

        // Initialization
        JFrame jf = new JFrame("VisDemo");
        jf.setSize(640,480);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        relPose = new double[]{0,0,0,0,0,0};
        frameSize = 1;

        final VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);

        ParameterGUI pg = new ParameterGUI();
        pg.addDoubleSlider("tranX","X Translationx",-10,10,0);
        pg.addDoubleSlider("tranY","Y Translationx",-10,10,0);
        pg.addDoubleSlider("tranZ","Z Translationx",-10,10,0);
        pg.addDoubleSlider("rotX","X Rotation",-Math.PI,Math.PI,0);
        pg.addDoubleSlider("rotY","Y Rotation",-Math.PI,Math.PI,0);
        pg.addDoubleSlider("rotZ","Z Rotation",-Math.PI,Math.PI,0);
        pg.addDoubleSlider("frameSize","Frame Size",.1, 10, 1);

        jf.setLayout(new BorderLayout());
        jf.add(vc,BorderLayout.CENTER);
        jf.add(pg,BorderLayout.SOUTH);


        jf.setVisible(true);
        
        // Drawing code
        {
            VisGrid vg = new VisGrid(new Color(.5f,.5f,.5f),
                                     new Color(1.0f,1.0f,1.0f,0.0f));

            VisWorld.Buffer vb = vw.getBuffer("grid");
            vb.setDrawOrder(-100);
            vb.addBack(new VisDepthTest(false,vg));
            vb.swap();
        }

        pg.addListener(new ParameterListener(){
                public void parameterChanged(ParameterGUI pg, String name)
                {

                    if (name.equals("tranX")) {
                        relPose[0] = pg.gd("tranX");
                    }
                    if (name.equals("tranY")) {
                        relPose[1] = pg.gd("tranY");
                    }
                    if (name.equals("tranZ")) {
                        relPose[2] = pg.gd("tranZ");
                    }
                    if (name.equals("rotX")) {
                        relPose[3] = pg.gd("rotX");
                    }
                    if (name.equals("rotY")) {
                        relPose[4] = pg.gd("rotY");
                    }
                    if (name.equals("rotZ")) {
                        relPose[5] = pg.gd("rotZ");
                    }
                    if (name.equals("frameSize")) {
                        frameSize = pg.gd("frameSize");
                    }

                    VisWorld.Buffer vb = vw.getBuffer("quiver");

                    Quiver quiv = new Quiver();
                    vb.addBack(quiv.getQuiverAt(relPose, frameSize));
                    vb.swap();

                }
            });

        //Show the quiver at initial pose
        VisWorld.Buffer vb = vw.getBuffer("quiver");
        Quiver quiv = new Quiver();
        vb.addBack(quiv.getQuiverAt(relPose));
        vb.swap();

    }
}
