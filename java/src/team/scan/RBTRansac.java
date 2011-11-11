package team.scan;

import team.*;
import april.vis.*;
import java.util.*;

public class RBTRansac {

    private List<Corner> cornersA;
    private List<Corner> cornersB;

    private List<Line> linesA;
    private List<Line> linesB;
    
    public RBTRansac(ArrayList<double[]> pointsA, ArrayList<double[]> pointsB,
                     VisWorld.Buffer lineBuffA, VisWorld.Buffer lineBuffB,
                     int numIter, double threshold) {

        AggloLineFit lineFitterA = new AggloLineFit(pointsA, lineBuffA, numIter, threshold);
        AggloLineFit lineFitterB = new AggloLineFit(pointsB, lineBuffB, numIter, threshold);

        //get the lines
        this.linesA = lineFitterA.getLines();
        this.linesB = lineFitterB.getLines();

        //get the corners
        this.cornersA = Corner.getAllCorners(this.linesA);
        this.cornersB = Corner.getAllCorners(this.linesB);

    }

    //Bjarne would hate us but this returns x, y, and theta describing
    //2D RBT.  Use
    private double[] doRansac() {
        return null;
    }

    private double consensusScore() {
        //for points in a
        //for points in b
        //  compute dist from a to b, count num points inside circle
        return 0;
    }

}