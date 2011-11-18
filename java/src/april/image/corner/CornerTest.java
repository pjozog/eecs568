package april.image.corner;

import java.awt.*;
import java.awt.image.*;
import java.io.*;
import java.util.*;
import javax.swing.*;
import javax.imageio.*;

import april.util.*;
import april.image.*;
import april.vis.*;
import april.jmat.*;
import april.jcam.*;

public class CornerTest implements ParameterListener
{
    JFrame jf;
    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);
    ParameterGUI pg = new ParameterGUI();

    BufferedImage im;

    JTabbedPane jtp;
    ImageSource isrc;

    CornerDetector detectors[] = new CornerDetector[] { new KanadeDetector(),
                                                        new DoGDetector()};

    interface CornerDetector
    {
        public JPanel getConfigurationPanel();
        public FloatImage computeResponse(FloatImage input);
        public String getName();
    }

    public static void main(String args[])
    {
        if (args.length == 0) {
            System.out.println("Provide an image source URL, even file:///foo.jpg");
            System.exit(0);
        }

        try {
            ImageSource isrc = ImageSource.make(args[0]);
            new CornerTest(isrc);
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }

/*
        try {
            BufferedImage im = ImageIO.read(new File(args[0]));
            FloatImage fim = new FloatImage(im);
            fim = fim.interpolate(1.5, 5).normalize();
            new CornerTest(fim.makeImage());
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }
*/
    }

    public CornerTest(ImageSource isrc)
    {
        this.isrc = isrc;

        jf = new JFrame("CornerTest");
        jf.setLayout(new BorderLayout());

        pg.addDoubleSlider("thresh", "strength thresh", 0, 1, 0.01);
        pg.addListener(this);

        jtp = new JTabbedPane();
        for (int i = 0; i < detectors.length; i++)
            jtp.addTab(detectors[i].getName(), detectors[i].getConfigurationPanel());

        jf.add(vc, BorderLayout.CENTER);

        JPanel jp = new JPanel();
        jp.setLayout(new BorderLayout());
        jp.add(pg.getPanel(), BorderLayout.NORTH);
        jp.add(jtp, BorderLayout.SOUTH);
        jf.add(jp, BorderLayout.SOUTH);

        jf.setSize(800,600);
        jf.setVisible(true);

        vl.backgroundColor = new Color(128,128,128);
        ((DefaultCameraManager) vl.cameraManager).interfaceMode = 1.5;

        jf.add(new LayerBufferPanel(vc), BorderLayout.EAST);

        new RunThread().start();
    }

    class RunThread extends Thread
    {
        public void run()
        {
            ImageSourceFormat ifmt = isrc.getCurrentFormat();
            boolean first = true;

            isrc.start();

            while (true) {
                byte imbuf[] = isrc.getFrame();

                im = ImageConvert.convertToImage(ifmt.format, ifmt.width, ifmt.height, imbuf);
                if (im == null)
                    continue;

                if (first) {
                    vl.cameraManager.fit2D(new double[] {0, im.getHeight()}, new double[] {im.getWidth(), 0}, true);
                    first = false;
                }

                update();
            }
        }
    }

    public void parameterChanged(ParameterGUI pg, String name)
    {
        update();
    }

    public void update()
    {
        if (im == null)
            return;

        CornerDetector cd = detectors[jtp.getSelectedIndex()];

        FloatImage fim = new FloatImage(im);
        FloatImage response = cd.computeResponse(fim).normalize();

        BufferedImage out = response.makeColorImage();

        if (true) {
            VisWorld.Buffer vb = vw.getBuffer("original");
            vb.addBack(new VisChain(LinAlg.translate(0, im.getHeight(), 0),
                                    LinAlg.scale(1, -1, 1),
                                    new VzImage(im)));
            vb.swap();
        }

        if (true) {
            VisWorld.Buffer vb = vw.getBuffer("response");
            vb.addBack(new VisChain(LinAlg.translate(0, out.getHeight(), 0),
                                    LinAlg.scale(1, -1, 1),
                                    new VzImage(out)));
            vb.swap();
        }

        if (true) {
            VisWorld.Buffer vb = vw.getBuffer("corners");

            double thresh = pg.gd("thresh");

            ArrayList<float[]> corners = response.localMaxima();
            for (float corner[] : corners) {
                if (corner[2] < thresh)
                    continue;

                vb.addBack(new VisChain(LinAlg.translate(0, out.getHeight(), 0),
                                        LinAlg.scale(1, -1, 1),
                                        LinAlg.translate(corner[0], corner[1], 0),
                                        new VzCircle(4, Color.yellow)));
            }

            vb.swap();
        }
    }

    class KanadeDetector implements CornerDetector, ParameterListener
    {
        ParameterGUI pg;
        JPanel jp;

        KanadeDetector()
        {
            pg = new ParameterGUI();
            pg.addIntSlider("halfsize", "window size/2", 0, 10, 2);
            pg.addListener(this);

            jp = new JPanel();
            jp.setLayout(new BorderLayout());
            jp.add(pg, BorderLayout.CENTER);
        }

        public void parameterChanged(ParameterGUI pg, String name)
        {
            CornerTest.this.update();
        }

        public String getName()
        {
            return "Kanade-Tomasi";
        }

        public JPanel getConfigurationPanel()
        {
            return jp;
        }

        public FloatImage computeResponse(FloatImage input)
        {
            KanadeTomasi kt = new KanadeTomasi(pg.gi("halfsize"));
            FloatImage response = kt.computeResponse(input);

            return response;
        }
    }


    class DoGDetector implements CornerDetector, ParameterListener
    {
        ParameterGUI pg;
        JPanel jp;

        DoGDetector()
        {
            pg = new ParameterGUI();
            pg.addDoubleSlider("sigma1", "sigma1", 0, 10, 1);
            pg.addDoubleSlider("sigma2", "sigma2", 0, 10, 2);
            pg.addListener(this);

            jp = new JPanel();
            jp.setLayout(new BorderLayout());
            jp.add(pg, BorderLayout.CENTER);
        }

        public void parameterChanged(ParameterGUI pg, String name)
        {
            CornerTest.this.update();
        }

        public String getName()
        {
            return "DoG";
        }

        public JPanel getConfigurationPanel()
        {
            return jp;
        }

        public FloatImage computeResponse(FloatImage input)
        {
            DoG kt = new DoG(pg.gd("sigma1"), pg.gd("sigma2"));
            FloatImage response = kt.computeResponse(input);

            return response;
        }
    }

}
