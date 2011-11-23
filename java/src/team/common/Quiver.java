package team.common;

import java.awt.*;
import java.util.*;
import april.vis.*;
import april.jmat.*;

public class Quiver {
    
    private static double[] origPnt  = new double[]{0,0,0,1};
    private static double[] xAxisPnt = new double[]{1,0,0,1};
    private static double[] yAxisPnt = new double[]{0,1,0,1};
    private static double[] zAxisPnt = new double[]{0,0,1,1};
    
    public static ArrayList<double[]> points;

    private static int rgbToBgr(int color) {
        return 0xFF000000 | (color & 0x000000FF) << 16 | 
            (color & 0x0000FF00) | 
            (color & 0x00FF0000) >> 16;
    }

    public static VzLines getQuiverAt(double[] xyzrpy, double scale, Color xColor, Color yColor, Color zColor) {
        
        assert(xyzrpy.length == 6);

        int xColorInt = rgbToBgr(xColor.getRGB());
        int yColorInt = rgbToBgr(yColor.getRGB());
        int zColorInt = rgbToBgr(zColor.getRGB());

        Matrix transform = new Matrix(LinAlg.xyzrpyToMatrix(xyzrpy));

        xAxisPnt[0] = scale;
        yAxisPnt[1] = scale;
        zAxisPnt[2] = scale;

        ArrayList<double[]> quiverPnts = new ArrayList<double[]>();
        quiverPnts.add(transform.times(origPnt));
        quiverPnts.add(transform.times(xAxisPnt));
        quiverPnts.add(transform.times(origPnt));
        quiverPnts.add(transform.times(yAxisPnt));
        quiverPnts.add(transform.times(origPnt));
        quiverPnts.add(transform.times(zAxisPnt));

        VisColorData vcd = new VisColorData();
        vcd.add(xColorInt);
        vcd.add(xColorInt);
        vcd.add(yColorInt);
        vcd.add(yColorInt);
        vcd.add(zColorInt);
        vcd.add(zColorInt);

        VzLines quiver = new VzLines(new VisVertexData(quiverPnts), vcd, 3, VzLines.TYPE.LINES);
        
        return quiver;

    }

    public static VzLines getQuiverAt(double[] xyzrpy, double scale) {
        
        return getQuiverAt(xyzrpy, scale, Color.red, Color.green, Color.blue);

    }

    public static VzLines getQuiverAt(double[] xyzrpy) {

        return getQuiverAt(xyzrpy, 1.0);

    }

}
