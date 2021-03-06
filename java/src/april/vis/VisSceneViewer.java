package april.vis;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.*;
import java.io.*;
import java.util.*;
import java.util.zip.*;
import javax.swing.*;
import javax.imageio.*;

import april.jmat.*;
import april.jmat.geom.*;

public class VisSceneViewer
{
    JFrame jf;

    public VisSceneViewer(String name, VisCanvas vc)
    {
        jf = new JFrame(name);
        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.setSize(600,400);
        jf.setVisible(true);
    }

    public static void main(String args[])
    {
        try {
            long time0 = System.currentTimeMillis();
            ObjectReader ins = new ObjectReader(args[0]);
            VisCanvas vc = (VisCanvas) ins.readObject();
            long time1 = System.currentTimeMillis();

            System.out.printf("Time to load scene: %15.3f s\n", (time1 - time0) / 1000.0);
            new VisSceneViewer(args[0], vc);
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }
    }
}
