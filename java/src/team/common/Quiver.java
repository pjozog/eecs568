package team.common;

import java.awt.*;
import java.util.*;
import april.vis.*;
import april.jmat.*;

public class Quiver {
    
    private VisLines vline;

    private double[] origPnt  = new double[]{0,0,0,1};
    private double[] xAxisPnt = new double[]{1,0,0,1};
    private double[] yAxisPnt = new double[]{0,1,0,1};
    private double[] zAxisPnt = new double[]{0,0,1,1};
    
    public ArrayList<double[]> points;

    public Quiver() {

    }

    public VisLines getQuiverAt(double[] xyzrpy, double scale) {
        
        assert(xyzrpy.length == 6);

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
        vcd.add(0xff0000ff);
        vcd.add(0xff0000ff);
        vcd.add(0xff00ff00);
        vcd.add(0xff00ff00);
        vcd.add(0xffff0000);
        vcd.add(0xffff0000);

        VisLines quiver = new VisLines(new VisVertexData(quiverPnts), vcd, 3, VisLines.TYPE.LINES);
        
        return quiver;

    }

    public VisLines getQuiverAt(double[] xyzrpy) {

        return getQuiverAt(xyzrpy, 1.0);

    }

}
