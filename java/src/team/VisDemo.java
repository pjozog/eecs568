package demo;

import javax.swing.*; // For JFrame
import java.awt.*; // For BorderLayout
import java.util.*;

import april.vis.*; // For VisCanvas etc
import april.util.*; // For Parameter GUI
import april.jmat.*;


public class VisDemo
{
    double v;

    public VisDemo(double v)
    {
        this.v = v;
    }


    public static void main(String args[])
    {
        VisDemo vdemo = new VisDemo(1.0);


        // Initialization
        JFrame jf = new JFrame("VisDemo");
        jf.setSize(640,480);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        final VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);

        ParameterGUI pg = new ParameterGUI();
        pg.addDoubleSlider("sig","Sigma",.0001,10,2.0);
        pg.addDoubleSlider("startx","Starting X position",-10,10,2.3);
        pg.addDoubleSlider("starty","Starting Y position",-10,10,-1);
        pg.addButtons("start","Start","stop","Stop","pause","Pause");


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
                    if (name.equals("startx") || name.equals("starty")) {
                        double pt[] = new double[]{pg.gd("startx"), pg.gd("starty")};

                        VisVertexData vd = new VisVertexData();
                        vd.add(pt);
                        VisConstantColor vcol = new VisConstantColor(Color.blue);

                        VisPoints vp = new VisPoints(vd, vcol, 15);
                        VisWorld.Buffer vb = vw.getBuffer("point");
                        vb.addBack(vp);
                        vb.swap();
                    }

                    if (name.equals("sig")) {
                        ArrayList<double[]> points  = new ArrayList<double[]>();
                        Random r = new Random(1);
                        double max = -Double.MAX_VALUE;
                        double min = Double.MAX_VALUE;

                        for (int i = 0; i < 100; i++) {

                            double pt[] = new double[]{r.nextGaussian()*pg.gd("sig"),
                                                    r.nextGaussian()*pg.gd("sig"),
                                                       r.nextGaussian()*pg.gd("sig")};


                            points.add(pt);


                            max = Math.max(max,pt[2]);
                            min = Math.min(min,pt[2]);

                        }

                        VisVertexData vd = new VisVertexData(points);


                        VisColorData vcd = new VisColorData();

                        for (double pt[] : points) {
                            double z = pt[2];

                            double ratio = (z-min) / (max-min);

                            int red = (int)(ratio*255);

                            int col = 0xff000000;
                            col = col | (red << 0);
                            vcd.add(col);
                        }


                        VisWorld.Buffer vb = vw.getBuffer("cloud");
                        vb.addBack(new VisPoints(vd,vcd, 4));
                        // vb.addBack(new VisLines(vd,vcd, 4, VisLines.TYPE.LINES));
                        vb.swap();
                    }
                }
            });







        // pg.notifyListeners("sig");

        int i = 0;
        while (true) {
            i++;
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {}



            ArrayList<double[]> points = new ArrayList<double[]>();
            points.add(new double[]{-1,-1});
            points.add(new double[]{1,-1});
            points.add(new double[]{1,1});
            points.add(new double[]{-1,1});


            VisLines vline = new VisLines(new VisVertexData(points),
                                       new VisConstantColor(Color.blue), 3, VisLines.TYPE.LINE_LOOP);

            VisWorld.Buffer vb = vw.getBuffer("box");

            VisChain box = new VisChain(LinAlg.translate(1,0,0),
                                        LinAlg.rotateY(Math.PI/2),
                                        vline,
                                        LinAlg.rotateY(-Math.PI/2),
                                        LinAlg.translate(-2,0,0),
                                        LinAlg.rotateY(Math.PI/2),
                                        vline,
                                        LinAlg.rotateY(-Math.PI/2),
                                        LinAlg.translate(1,1,0),
                                        LinAlg.rotateX(Math.PI/2),
                                        vline,
                                        LinAlg.rotateX(-Math.PI/2),
                                        LinAlg.translate(0,-2,0),
                                        LinAlg.rotateX(Math.PI/2),
                                        vline);

            double rad = 10;
            double theta = Math.toRadians(13)/100 * i;
            double pos[] = {rad*Math.cos(theta),rad*Math.sin(theta),0};
            vb.addBack(new VisChain(LinAlg.translate(pos),LinAlg.rotateZ(theta), box));
            vb.swap();


        }

        // System.out.println("Hello World");
    }
}












